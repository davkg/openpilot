"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import pickle
import requests

from openpilot.common.hardware import hw
from openpilot.common.test import OpenpilotTestCase
import openpilot.sunnypilot.modeld_v2.modeld as modeld_module
from openpilot.sunnypilot.modeld_v2.tests import helpers as tests_helpers
from openpilot.sunnypilot.modeld_v2.tests.helpers import DummyModel, DummyBundle, CAM_W, CAM_H
from openpilot.sunnypilot.models.fetcher import ModelParser, ModelFetcher

tmp_path = tests_helpers.tmp_path


class TestFallback(OpenpilotTestCase):
  def test_find_dual_model_in_bundle(self, tmp_path, monkeypatch):
    lebowski_file = 'driving_lebowski.pkl'
    tsfdo_file = 'driving_tsfdo.pkl'
    (tmp_path / lebowski_file).write_bytes(b'fkasdjfkljf')
    (tmp_path / tsfdo_file).write_bytes(b'dskfajklsdjlsfka')

    monkeypatch.setattr(hw.Paths, 'model_root', staticmethod(lambda: str(tmp_path)))
    big_bundle = DummyBundle(models=[DummyModel('supercombo', lebowski_file)])
    small_bundle = DummyBundle(models=[DummyModel('supercombo', tsfdo_file)])
    big_pkl = modeld_module._find_driving_pkl(big_bundle)
    small_pkl = modeld_module._find_driving_pkl(small_bundle)

    assert big_pkl is not None and lebowski_file in big_pkl
    assert small_pkl is not None and tsfdo_file in small_pkl

  def test_download_models_and_init_modelstate_fallback(self, tmp_path, monkeypatch):
    monkeypatch.setattr(hw.Paths, 'model_root', staticmethod(lambda: str(tmp_path)))
    big_json = requests.get(ModelFetcher.MODEL_URL_CHESTNUT).json()
    big_bundle = ModelParser.parse_models(big_json)[-1]
    small_json = requests.get(ModelFetcher.MODEL_URL).json()
    small_bundle = ModelParser.parse_models(small_json)[-1]

    oob_bytes = pickle.dumps(tests_helpers.make_pkl_data(tests_helpers.ARCHETYPES['supercombo_non20hz']))
    monkeypatch.setattr(modeld_module, 'load_model', lambda path, **_: pickle.loads(open(path, 'rb').read()))

    for bundle in (big_bundle, small_bundle):
      (tmp_path / bundle.models[0].artifact.fileName).write_bytes(oob_bytes)

    monkeypatch.setattr(modeld_module, 'get_active_bundle', lambda params=None, *, chestnut=None: small_bundle)
    assert modeld_module.ModelState(CAM_W, CAM_H, chestnut=False).chestnut is False

    monkeypatch.setattr(modeld_module, 'get_active_bundle', lambda params=None, *, chestnut=None: big_bundle)
    try:
      assert modeld_module.ModelState(CAM_W, CAM_H, chestnut=True).chestnut is True
    except Exception as e:
      assert "AMD" in str(e) or "device" in str(e).lower()


class TestZeroNonfinite(OpenpilotTestCase):
  def test_zeroes_in_place(self):
    import numpy as np
    from tinygrad.tensor import Tensor
    from openpilot.sunnypilot.modeld_v2.model_adapters import input_view, zero_nonfinite

    feat = np.arange(12, dtype=np.float32).reshape(3, 4)
    feat[2, 1], feat[0, 3] = np.nan, np.inf
    queues = {
      'feat_q': Tensor(feat, device='CPU').contiguous().realize(),
      'clean_q': Tensor(np.ones((2, 2), dtype=np.float32), device='CPU').contiguous().realize(),
      'img_q': Tensor(np.full(4, 255, dtype=np.uint8), device='CPU').contiguous().realize(),
    }
    # the jit reads the state through views of the same buffer, so the fix must land in that buffer
    alias = input_view(queues['feat_q']._buffer(), (3, 4), queues['feat_q'].dtype, 0)

    assert zero_nonfinite(queues) == ['feat_q']
    expected = feat.copy()
    expected[[0, 2]] = 0  # whole slots: an overflowed slot's finite values are suspect too
    np.testing.assert_array_equal(alias.numpy(), expected)
    np.testing.assert_array_equal(queues['feat_q'].numpy(), expected)
    assert zero_nonfinite(queues) == []


class TestSkipBadFrame(OpenpilotTestCase):
  def _run(self, events):
    from collections import deque
    times: deque[float] = deque(maxlen=modeld_module.MAX_NONFINITE_FRAMES)
    return [modeld_module.skip_bad_frame(times, t, nonfinite) for t, nonfinite in events]

  def test_single_glitch_skips(self):
    assert self._run([(1.0, True)]) == [True]

  def test_three_in_a_row_falls_back(self):
    assert self._run([(1.0, True), (1.05, True), (1.1, True)]) == [True, True, False]

  def test_intermittent_falls_back(self):
    # bad, good, bad, good, bad: a reset-on-good counter would never fall back
    assert self._run([(1.0, True), (1.1, True), (1.2, True)]) == [True, True, False]

  def test_isolated_glitches_keep_big_model(self):
    w = modeld_module.NONFINITE_WINDOW
    assert all(self._run([(i * w / 1.5, True) for i in range(6)]))

  def test_other_errors_fall_back_at_once(self):
    assert self._run([(1.0, False)]) == [False]
