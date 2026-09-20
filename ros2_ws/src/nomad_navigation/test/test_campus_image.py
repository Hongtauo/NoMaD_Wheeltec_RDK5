"""Bounded CPU-only JPEG preparation; no ROS, server, GPU, or robot access."""

import base64
from collections import Counter
import copy
import io
import subprocess
import sys
import unittest
from unittest.mock import patch
import uuid

from nomad_navigation.campus_image import (
    ImagePreparationError, UploadImagePreparer, compact_jpeg,
)
from nomad_navigation.campus_protocol import (
    MAX_JPEG_BYTES, ProtocolError, decode_jpeg_bytes, validate_request,
)


def jpeg_fixture(size=(640, 480), color=(80, 120, 160), mode='RGB', **kwargs):
    from PIL import Image
    buffer = io.BytesIO()
    Image.new(mode, size, color).save(buffer, 'JPEG', quality=95, **kwargs)
    return buffer.getvalue()


def request_fixture(images=None):
    if images is None:
        images = [jpeg_fixture(color=(20 + n * 30, 60, 120)) for n in range(4)]
    return dict(version=1, session_id=str(uuid.uuid4()), sequence=1,
                frame_id='camera_color_optical_frame', observation_now_ns=10_800_000_000,
                frames=[dict(stamp_ns=10_000_000_000 + n * 250_000_000,
                             jpeg_b64=base64.b64encode(jpeg).decode('ascii'))
                        for n, jpeg in enumerate(images)],
                telemetry={'controller_status': {'state': 'disarmed'}})


class CampusImageTests(unittest.TestCase):
    def test_import_does_not_load_pillow_ros_or_ml(self):
        result = subprocess.run([sys.executable, '-c',
            'import sys; import nomad_navigation.campus_image; '
            'assert not any(k in sys.modules for k in ("PIL", "cv2", "torch", "numpy", "rclpy"))'],
            check=False, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_landscape_portrait_and_no_upscale(self):
        from PIL import Image
        for size, expected in [((640, 480), (320, 240)), ((480, 640), (240, 320)),
                               ((128, 64), (128, 64)), ((641, 479), (320, 239))]:
            output = compact_jpeg(jpeg_fixture(size=size))
            with Image.open(io.BytesIO(output)) as image:
                self.assertEqual(image.size, expected)
                self.assertEqual(image.mode, 'RGB')
                self.assertEqual(image.format, 'JPEG')
            self.assertLessEqual(len(output), MAX_JPEG_BYTES)

    def test_complete_field_of_view_kept_no_center_crop(self):
        from PIL import Image, ImageDraw
        source = Image.new('RGB', (640, 480), (0, 0, 0))
        draw = ImageDraw.Draw(source)
        draw.rectangle((0, 0, 79, 479), fill=(240, 10, 10))
        draw.rectangle((560, 0, 639, 479), fill=(10, 240, 10))
        draw.rectangle((100, 0, 540, 39), fill=(10, 10, 240))
        draw.rectangle((100, 440, 540, 479), fill=(230, 230, 10))
        stream = io.BytesIO()
        source.save(stream, 'JPEG', quality=95)
        with Image.open(io.BytesIO(compact_jpeg(stream.getvalue()))) as result:
            self.assertGreater(result.getpixel((10, 120))[0], 200)
            self.assertGreater(result.getpixel((310, 120))[1], 200)
            self.assertGreater(result.getpixel((160, 5))[2], 200)
            self.assertGreater(result.getpixel((160, 230))[0], 190)
            self.assertGreater(result.getpixel((160, 230))[1], 190)

    def test_exif_orientation_not_applied_like_existing_server(self):
        from PIL import Image
        exif = Image.Exif()
        exif[274] = 6
        output = compact_jpeg(jpeg_fixture(exif=exif))
        with Image.open(io.BytesIO(output)) as image:
            self.assertEqual(image.size, (320, 240))
            self.assertNotIn(274, image.getexif())

    def test_grayscale_and_cmyk_become_rgb(self):
        from PIL import Image
        for mode, color in [('L', 100), ('CMYK', (0, 30, 60, 0))]:
            with Image.open(io.BytesIO(compact_jpeg(jpeg_fixture(mode=mode, color=color)))) as image:
                self.assertEqual(image.mode, 'RGB')

    def test_malformed_and_truncated_jpeg_rejected(self):
        valid = jpeg_fixture()
        for raw in (b'', 'not bytes', b'not-jpeg', b'\xff\xd8garbage\xff\xd9',
                    valid[:100] + b'\xff\xd9', valid[:-2]):
            with self.assertRaises(ImagePreparationError):
                compact_jpeg(raw)

    def test_byte_dimension_pixel_and_extreme_aspect_bounds(self):
        with self.assertRaises(ImagePreparationError):
            compact_jpeg(b'\xff\xd8' + b'x' * MAX_JPEG_BYTES + b'\xff\xd9')
        for size in [(1930, 16), (1920, 1200), (15, 480), (16, 1920)]:
            with self.subTest(size=size), self.assertRaises(ImagePreparationError):
                compact_jpeg(jpeg_fixture(size=size))

    def test_oversized_header_rejected_before_pixel_decode(self):
        from PIL import Image
        source = unittest.mock.MagicMock()
        source.format, source.size = 'JPEG', (1920, 1920)
        source.__enter__.return_value = source
        with patch.object(Image, 'open', return_value=source):
            with self.assertRaises(ImagePreparationError):
                compact_jpeg(b'\xff\xd8x\xff\xd9')
        source.load.assert_not_called()

    def test_encoded_output_byte_limit(self):
        from PIL import Image
        source = jpeg_fixture()
        def oversized(_image, output, *args, **kwargs):
            output.write(b'\xff\xd8' + b'x' * MAX_JPEG_BYTES + b'\xff\xd9')
        with patch.object(Image.Image, 'save', new=oversized):
            with self.assertRaises(ImagePreparationError):
                compact_jpeg(source, backend='pillow')

    def test_only_reviewed_quality_edge_and_cache_bounds(self):
        source = jpeg_fixture()
        for kwargs in ({'quality': 0}, {'quality': 95}, {'quality': True},
                       {'max_edge': 96}, {'max_edge': True}, {'max_edge': 640}):
            with self.assertRaises(ValueError):
                compact_jpeg(source, **kwargs)
        for entries in (-1, 9, True, 1.5):
            with self.assertRaises(ValueError):
                UploadImagePreparer(max_cache_entries=entries)
        self.assertTrue(compact_jpeg(source, quality=75))

    def test_request_metadata_original_stamps_and_input_are_unchanged(self):
        request = request_fixture()
        original = copy.deepcopy(request)
        preparer = UploadImagePreparer()
        output = preparer.prepare_request(request)
        self.assertEqual(request, original)
        self.assertIsNot(output, request)
        self.assertIsNot(output['telemetry'], request['telemetry'])
        self.assertIs(validate_request(output), output)
        for name in request.keys() - {'frames'}:
            self.assertEqual(output[name], request[name])
        self.assertEqual([f['stamp_ns'] for f in output['frames']], [f['stamp_ns'] for f in request['frames']])
        self.assertNotEqual(output['frames'][0]['jpeg_b64'], request['frames'][0]['jpeg_b64'])
        self.assertEqual(preparer.last_metrics['cache_misses'], 4)
        self.assertTrue(preparer.last_metrics['complete'])
        self.assertGreater(preparer.last_metrics['source_jpeg_bytes'], preparer.last_metrics['encoded_jpeg_bytes'])
        self.assertGreaterEqual(preparer.last_metrics['prepare_ms'], 0.)
        # Diagnostics stay out of the strict wire protocol.
        self.assertNotIn('prepare_ms', output)

    def test_cache_reuses_content_without_caching_old_timestamps(self):
        request = request_fixture()
        preparer = UploadImagePreparer()
        preparer.prepare_request(request)
        for frame in request['frames']:
            frame['stamp_ns'] += 250_000_000
        request['observation_now_ns'] += 250_000_000
        request['sequence'] = 2
        with patch('nomad_navigation.campus_image.compact_jpeg', side_effect=AssertionError('cache miss')):
            output = preparer.prepare_request(request)
        self.assertEqual(preparer.last_metrics['cache_hits'], 4)
        self.assertEqual(output['frames'][0]['stamp_ns'], request['frames'][0]['stamp_ns'])
        self.assertEqual(output['sequence'], 2)

    def test_legacy_inputs_decode_once_and_generated_output_is_not_decoded_here(self):
        request = request_fixture()
        originals = [frame['jpeg_b64'] for frame in request['frames']]
        preparer = UploadImagePreparer()
        for expected_hits in (0, 4):
            with patch('nomad_navigation.campus_protocol.decode_jpeg_bytes',
                       wraps=decode_jpeg_bytes) as decode:
                output = preparer.prepare_request(request)
            decoded_arguments = Counter(call.args[0] for call in decode.call_args_list)
            encoded_output = [frame['jpeg_b64'] for frame in output['frames']]
            self.assertTrue(set(originals).isdisjoint(encoded_output))
            self.assertEqual(decoded_arguments, Counter(originals))
            self.assertEqual(decode.call_count, 4)
            self.assertIs(validate_request(output), output)
            self.assertEqual(preparer.last_metrics['cache_hits'], expected_hits)
            self.assertTrue(preparer.last_metrics['complete'])

    def test_full_request_validation_precedes_every_codec_and_cache_update(self):
        mutations = [
            lambda p: p.update(session_id='not-a-uuid'),
            lambda p: p.update(frame_id='wrong-camera'),
            lambda p: p['frames'][-1].update(jpeg_b64='***'),
            lambda p: p['frames'][-1].update(jpeg_b64='A' * (4 * ((MAX_JPEG_BYTES + 2) // 3) + 1)),
            lambda p: p['frames'][-1].update(stamp_ns=p['frames'][-2]['stamp_ns']),
            lambda p: p.update(observation_now_ns=p['frames'][-1]['stamp_ns'] + 1_000_000_001),
            lambda p: p.update(telemetry={'controller_status': {'data': 'x' * 32768}}),
        ]
        for index, mutate in enumerate(mutations):
            request = request_fixture()
            mutate(request)
            preparer = UploadImagePreparer()
            with self.subTest(mutation=index), patch(
                    'nomad_navigation.campus_image.compact_jpeg',
                    side_effect=AssertionError('codec before full validation')) as codec:
                with self.assertRaises(ProtocolError):
                    preparer.prepare_request(request)
                codec.assert_not_called()
            self.assertEqual(len(preparer._cache), 0)
            self.assertFalse(preparer.last_metrics['complete'])

    def test_invalid_owned_cache_entry_is_rejected(self):
        request = request_fixture()
        preparer = UploadImagePreparer()
        preparer.prepare_request(request)
        for key in preparer._cache:
            preparer._cache[key] = b'not-a-jpeg'
        with patch('nomad_navigation.campus_image.compact_jpeg',
                   side_effect=AssertionError('expected cache hit')):
            with self.assertRaises(ProtocolError):
                preparer.prepare_request(request)
        self.assertFalse(preparer.last_metrics['complete'])

    def test_protocol_valid_envelope_does_not_bypass_codec_dimension_or_format_guards(self):
        invalid_sources = [b'\xff\xd8garbage\xff\xd9',
                           jpeg_fixture(size=(1920, 1200)), jpeg_fixture(size=(15, 480))]
        for source in invalid_sources:
            request = request_fixture([source] * 4)
            self.assertIs(validate_request(request), request)
            preparer = UploadImagePreparer()
            with self.assertRaises(ImagePreparationError):
                preparer.prepare_request(request)
            self.assertFalse(preparer.last_metrics['complete'])

    def test_cache_is_bounded_can_be_disabled_and_metrics_are_copied(self):
        for limit in (0, 2, 8):
            preparer = UploadImagePreparer(max_cache_entries=limit)
            self.assertIsNone(preparer.last_metrics)
            preparer.prepare_request(request_fixture())
            self.assertLessEqual(len(preparer._cache), limit)
            metrics = preparer.last_metrics
            metrics['cache_entries'] = 999
            self.assertNotEqual(preparer.last_metrics['cache_entries'], 999)
            preparer.prepare_request(request_fixture())
            self.assertLessEqual(len(preparer._cache), limit)

    def test_invalid_or_stale_request_cannot_be_refreshed_by_preparation(self):
        request = request_fixture()
        request['observation_now_ns'] += 2_000_000_000
        preparer = UploadImagePreparer()
        with self.assertRaises(ProtocolError):
            preparer.prepare_request(request)
        self.assertFalse(preparer.last_metrics['complete'])
        self.assertEqual(preparer.last_metrics['source_jpeg_bytes'], 0)

    def test_warmup_loads_codec_once_without_caching_observations(self):
        preparer = UploadImagePreparer()
        result = preparer.warmup()
        self.assertTrue(result['complete'])
        self.assertEqual(result['backend'], 'opencv')
        self.assertGreaterEqual(result['warmup_ms'], 0.)
        self.assertIsNone(preparer.last_metrics)
        self.assertEqual(len(preparer._cache), 0)
        with patch('nomad_navigation.campus_image.compact_jpeg', side_effect=AssertionError('second warmup')):
            self.assertEqual(preparer.warmup(), result)
        result['complete'] = False
        self.assertTrue(preparer.warmup()['complete'])

    def test_opencv_uses_reduced_decode_and_ignores_exif(self):
        import cv2
        source = jpeg_fixture()
        real_decode = cv2.imdecode
        calls = []
        def record(data, flags):
            calls.append(flags)
            return real_decode(data, flags)
        with patch.object(cv2, 'imdecode', side_effect=record):
            compact_jpeg(source)
        self.assertEqual(calls, [cv2.IMREAD_REDUCED_COLOR_2 | cv2.IMREAD_IGNORE_ORIENTATION])

    def test_opencv_header_guard_runs_before_decode(self):
        import cv2
        source = jpeg_fixture(size=(1920, 1200))
        with patch.object(cv2, 'imdecode', side_effect=AssertionError('header guard bypassed')):
            with self.assertRaises(ImagePreparationError):
                compact_jpeg(source)

    def test_opencv_decoded_bounds_dtype_and_failed_encode(self):
        import cv2
        import numpy as np
        source = jpeg_fixture()
        for bad in (None, np.zeros((240, 320), dtype=np.uint8),
                    np.zeros((240, 320, 3), dtype=np.float32),
                    np.zeros((480, 640, 3), dtype=np.uint8)):
            with patch.object(cv2, 'imdecode', return_value=bad):
                with self.assertRaises(ImagePreparationError):
                    compact_jpeg(source)
        with patch.object(cv2, 'imencode', return_value=(False, None)):
            with self.assertRaises(ImagePreparationError):
                compact_jpeg(source)
        with patch.object(cv2, 'imencode', return_value=(True, np.zeros(MAX_JPEG_BYTES+1, dtype=np.uint8))):
            with self.assertRaises(ImagePreparationError):
                compact_jpeg(source)

    def test_draft_and_full_pillow_backends_keep_size_and_frame_metadata(self):
        from PIL import Image
        request = request_fixture()
        for backend in ('pillow', 'pillow_draft', 'opencv'):
            preparer = UploadImagePreparer(backend=backend)
            preparer.warmup()
            output = preparer.prepare_request(request)
            self.assertEqual(output['frames'][0]['stamp_ns'], request['frames'][0]['stamp_ns'])
            with Image.open(io.BytesIO(base64.b64decode(output['frames'][0]['jpeg_b64']))) as image:
                self.assertEqual(image.size, (320, 240))
        with self.assertRaises(ValueError):
            UploadImagePreparer(backend='unknown')


if __name__ == '__main__':
    unittest.main()
