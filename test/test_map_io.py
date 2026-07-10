"""Tests for nav2-format map loading."""

from pathlib import Path

import cv2

from map2sdf.map_io import load_map

import numpy as np

import pytest

SAMPLE_YAML = Path(__file__).parent / 'maps' / 'sample.yaml'


def write_map(tmp_path, pixels, yaml_extra='', name='map'):
    cv2.imwrite(str(tmp_path / f'{name}.pgm'), pixels.astype(np.uint8))
    yaml_path = tmp_path / f'{name}.yaml'
    yaml_path.write_text(
        f'image: {name}.pgm\n'
        'resolution: 0.05\n'
        'origin: [-1.0, -2.0, 0.5]\n'
        'free_thresh: 0.196\n'  # keeps gray (205) in the unknown band
        + yaml_extra)
    return yaml_path


def test_trinary_thresholds(tmp_path):
    pixels = np.array([[0, 205, 254]])  # wall, unknown, free
    data = load_map(write_map(tmp_path, pixels))
    assert data.occupied.tolist() == [[True, False, False]]
    assert data.resolution == pytest.approx(0.05)
    assert data.origin == pytest.approx((-1.0, -2.0, 0.5))


def test_unknown_as_occupied(tmp_path):
    pixels = np.array([[0, 205, 254]])
    data = load_map(write_map(tmp_path, pixels), unknown_as='occupied')
    assert data.occupied.tolist() == [[True, True, False]]


def test_negate(tmp_path):
    pixels = np.array([[0, 254]])  # negated: bright pixels are occupied
    data = load_map(write_map(tmp_path, pixels, yaml_extra='negate: 1\n'))
    assert data.occupied.tolist() == [[False, True]]


def test_occupied_thresh_override(tmp_path):
    pixels = np.array([[100]])  # p = 0.608: free at 0.65, occupied at 0.5
    yaml_path = write_map(tmp_path, pixels)
    assert not load_map(yaml_path).occupied[0, 0]
    assert load_map(yaml_path, occupied_thresh=0.5).occupied[0, 0]


def test_missing_key_raises(tmp_path):
    yaml_path = tmp_path / 'bad.yaml'
    yaml_path.write_text('image: map.pgm\n')
    with pytest.raises(ValueError, match='resolution'):
        load_map(yaml_path)


def test_sample_map_loads():
    data = load_map(SAMPLE_YAML)
    assert data.size_cells == (100, 120)
    assert data.occupied[0, 0]        # outer wall
    assert not data.occupied[50, 30]  # free space
    assert not data.occupied[90, 10]  # unknown treated as free by default
    assert load_map(SAMPLE_YAML, unknown_as='occupied').occupied[90, 10]
