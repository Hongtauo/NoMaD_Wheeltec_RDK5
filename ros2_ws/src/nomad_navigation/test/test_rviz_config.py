"""Guard Humble's serialized Topic keys (old Image/Marker Topic keys are ignored)."""
from pathlib import Path
import yaml


def test_humble_rviz_topics_and_qos():
    path = Path(__file__).resolve().parents[1] / 'rviz' / 'nomad_preview.rviz'
    config = yaml.safe_load(path.read_text())['Visualization Manager']
    displays = {item['Class']: item for item in config['Displays']}
    image = displays['rviz_default_plugins/Image']['Topic']
    marker = displays['rviz_default_plugins/MarkerArray']['Topic']
    assert image['Value'] == '/camera/image_raw'
    assert image['Reliability Policy'] == 'Best Effort'
    assert marker['Value'] == '/nomad/visualization'
    assert marker['Reliability Policy'] == 'Reliable'
    assert config['Global Options']['Fixed Frame'] == 'base_link'
