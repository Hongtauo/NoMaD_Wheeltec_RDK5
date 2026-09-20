"""Read-only local RViz display of API predictions, never a control publisher."""
import base64
import io
import threading
import time

from campus_preview_snapshot import PREDICTION_LABEL, SnapshotReader


def decode_preview_image(encoded):
    """Decode in the viewer, never in the inference callback."""
    from PIL import Image
    with Image.open(io.BytesIO(base64.b64decode(encoded, validate=True))) as source:
        if (source.format != 'JPEG' or not 16 <= source.width <= 1920
                or not 16 <= source.height <= 1920 or source.width * source.height > 1920 * 1080):
            raise ValueError('Unsupported preview JPEG dimensions')
        rgb = source.convert('RGB')
        return rgb.width, rgb.height, rgb.tobytes()


class CampusPreview:
    def __init__(self, snapshot_path=None):
        self._lock = threading.Lock()
        self._latest = None
        self._stop = threading.Event()
        self._ready = threading.Event()
        self.error = None
        self.warning = None
        self.source = SnapshotReader(snapshot_path) if snapshot_path is not None else None
        self._thread = threading.Thread(target=self._run, name='campus-rviz', daemon=True)
        self._thread.start()
        if not self._ready.wait(10) or self.error:
            raise RuntimeError(f'Local RViz publisher failed: {self.error}')

    def submit(self, request, response, decoded_frames):
        # Called by inference: no decoding, disk I/O, or ROS blocking here.
        # Exactly one newest pending preview, no unbounded frame queue.
        with self._lock:
            self._latest = (request['frames'][-1], response, time.monotonic())

    def close(self):
        self._stop.set()
        self._thread.join(timeout=3)

    def _run(self):
        import os
        node = context = executor = None
        try:
            if os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
                raise RuntimeError('Campus preview is restricted to local-only ROS domain 87')
            import rclpy
            from rclpy.context import Context
            from rclpy.node import Node
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.signals import SignalHandlerOptions
            from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
            from sensor_msgs.msg import Image
            from geometry_msgs.msg import Point, TransformStamped
            from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
            from visualization_msgs.msg import Marker, MarkerArray
            context = Context()
            rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
            node = Node('campus_preview', namespace='/nomad', context=context,
                        use_global_arguments=False)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            # Explicit display-only identity transform, confined to domain 87.
            # This is NOT measured odometry or a robot/world localization TF.
            display_tf = StaticTransformBroadcaster(node)
            transform = TransformStamped()
            transform.header.stamp = node.get_clock().now().to_msg()
            transform.header.frame_id = 'nomad_preview_origin'
            transform.child_frame_id = 'base_link'
            transform.transform.rotation.w = 1.
            display_tf.sendTransform(transform)
            images = node.create_publisher(Image, '/nomad/campus_preview/image',
                                           qos_profile_sensor_data)
            markers = node.create_publisher(MarkerArray, '/nomad/campus_preview/markers',
                                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self._ready.set()
            last = None
            stale_sent = False
            snapshot_id = None
            while context.ok() and not self._stop.is_set():
                executor.spin_once(timeout_sec=.03)
                if self.source is not None:
                    try:
                        snapshot = self.source.latest()
                        self.warning = None
                    except Exception as error:
                        self.warning = type(error).__name__
                        snapshot = None
                    if snapshot is not None and snapshot['kind'] == 'prediction':
                        key = (snapshot['writer_id'], snapshot['published_ns'], snapshot['prediction']['sequence'])
                        if key != snapshot_id:
                            snapshot_id = key
                            with self._lock:
                                source_age = max(0., (time.time_ns() - snapshot['published_ns']) / 1e9)
                                self._latest = (snapshot['frame'], snapshot['prediction'], time.monotonic() - source_age)
                with self._lock:
                    item, self._latest = self._latest, None
                if item is None:
                    if (last is None or time.monotonic() - last > 1) and not stale_sent:
                        clear = Marker()
                        clear.action = Marker.DELETEALL
                        label = Marker()
                        label.header.frame_id = 'base_link'
                        label.header.stamp = node.get_clock().now().to_msg()
                        label.ns, label.id, label.type, label.action = 'campus_api_prediction', 22, Marker.TEXT_VIEW_FACING, Marker.ADD
                        label.pose.orientation.w = 1.
                        label.pose.position.x, label.pose.position.z = -.14, .06
                        label.scale.z = .035
                        label.color.r, label.color.g, label.color.a = 1., .65, 1.
                        label.text = ('STALE / WAITING — ' + PREDICTION_LABEL + '\n'
                                      'No fresh camera/prediction | Client acceptance/motion: UNKNOWN')
                        markers.publish(MarkerArray(markers=[clear, label]))
                        # An Image display otherwise retains the last camera
                        # frame indefinitely after a network or server stop.
                        blank = Image()
                        blank.header.stamp = node.get_clock().now().to_msg()
                        blank.header.frame_id = 'camera_color_optical_frame'
                        blank.height = blank.width = 2
                        blank.encoding, blank.step = 'rgb8', 6
                        blank.data = bytes([28, 32, 40] * 4)
                        images.publish(blank)
                        stale_sent = True
                    continue
                frame, result, arrival = item
                if time.monotonic() - arrival > .6:
                    continue
                last, stale_sent = arrival, False
                width, height, pixels = decode_preview_image(frame['jpeg_b64'])
                image = Image()
                # Original camera timestamp is retained for image metadata.
                image.header.stamp.sec, image.header.stamp.nanosec = divmod(frame['stamp_ns'], 10**9)
                image.header.frame_id = 'camera_color_optical_frame'
                image.width, image.height, image.encoding, image.step = width, height, 'rgb8', width * 3
                image.data = pixels
                images.publish(image)
                output = []
                clear = Marker()
                clear.action = Marker.DELETEALL
                output.append(clear)
                # Markers use the PC render time only in isolated domain 87.
                # They are explicitly server predictions, NOT accepted commands.
                render_stamp = node.get_clock().now().to_msg()
                def marker(identifier, kind):
                    value = Marker()
                    value.header.frame_id = 'base_link'
                    value.header.stamp = render_stamp
                    value.ns = 'campus_api_prediction'
                    value.id, value.type, value.action = identifier, kind, Marker.ADD
                    value.pose.orientation.w = 1.
                    value.color.a = 1.
                    value.lifetime.sec = 1
                    return value
                for index, trajectory in enumerate(result['trajectories']):
                    line = marker(index, Marker.LINE_STRIP)
                    line.scale.x = .007
                    line.color.r = 0.
                    line.color.g = .9 if index == 0 else .45
                    line.color.b = .6 if index == 0 else 1.
                    line.points = [Point(x=0., y=0., z=0.)] + [
                        Point(x=float(x), y=float(y), z=0.) for x, y in trajectory]
                    output.append(line)
                target = marker(20, Marker.SPHERE)
                target.scale.x = target.scale.y = target.scale.z = .04
                target.color.r, target.color.g = 1., .85
                target.pose.position.x, target.pose.position.y = map(float, result['waypoint'])
                output.append(target)
                origin = marker(21, Marker.CUBE)
                origin.scale.x, origin.scale.y, origin.scale.z = .10, .08, .025
                origin.color.r = origin.color.g = origin.color.b = .65
                output.append(origin)
                label = marker(22, Marker.TEXT_VIEW_FACING)
                label.scale.z = .035
                label.pose.position.x = -.14
                label.pose.position.z = .06
                label.color.r = label.color.g = label.color.b = 1.
                label.text = ('CAMPUS API | ' + PREDICTION_LABEL + '\n'
                              f"inference={result['inference_ms']:.0f} ms  seq={result['sequence']}\n"
                              'Client acceptance/motion: UNKNOWN | No odometry')
                output.append(label)
                markers.publish(MarkerArray(markers=output))
        except Exception as error:
            self.error = f'{type(error).__name__}: {error}'
            self._ready.set()
        finally:
            if executor is not None:
                executor.shutdown(timeout_sec=1)
            if node is not None:
                node.destroy_node()
            if context is not None and context.ok():
                context.shutdown()
