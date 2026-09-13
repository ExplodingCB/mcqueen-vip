import numpy as np

from mcq_training import labels as L
from mcq_training.boundary import extract_bounds, main_pavement
from mcq_training.metrics import BoundaryAccumulator, confusion_matrix, error_at_range, iou_per_class, mean_iou


def test_confusion_and_iou():
    gt = np.array([[0, 0, 1, 1], [1, 1, L.IGNORE, 2]])
    pred = np.array([[0, 1, 1, 1], [1, 0, 2, 2]])
    cm = confusion_matrix(pred, gt)
    assert cm.sum() == 7  # the ignore pixel is skipped
    assert cm[0, 0] == 1 and cm[0, 1] == 1 and cm[1, 1] == 3 and cm[1, 0] == 1 and cm[2, 2] == 1
    iou = iou_per_class(cm)
    assert np.isclose(iou[0], 1 / 3) and np.isclose(iou[1], 3 / 5) and np.isclose(iou[2], 1.0)
    assert np.isclose(mean_iou(cm), np.mean([1 / 3, 3 / 5, 1.0]))
    # A class absent everywhere is NaN and does not drag the mean down.
    cm2 = confusion_matrix(np.array([1, 1]), np.array([1, 1]))
    assert np.isnan(iou_per_class(cm2)[2]) and mean_iou(cm2) == 1.0


def test_main_component_drops_islands():
    m = np.zeros((10, 10), dtype=bool)
    m[5:, :] = True
    m[0, 0] = True
    keep = main_pavement(m)
    assert keep.sum() == 50 and not keep[0, 0]


def _pose(oval, s, d=0.0):
    x, y, psi = oval.cartesian(s, d)
    return float(x[0]), float(y[0]), float(psi[0])


def test_extracted_edges_land_on_the_survey(oval, cam):
    label = L.render_label(oval, _pose(oval, 10.0), cam, band_m=0.0, yaw_sigma=0.0)
    b = extract_bounds(label, cam)
    assert len(b) > 100
    # Where both edges are observed and within 15 m, they lie at +-2.5 m of the centerline.
    near = (b.left[:, 0] < 15.0) & (b.left_confidence > 0) & (b.right_confidence > 0)
    assert near.sum() > 50
    assert np.abs(b.left[near, 1] - 2.5).max() < 0.15
    assert np.abs(b.right[near, 1] + 2.5).max() < 0.15
    # The bottom rows see only pavement, so their edges are unobserved.
    assert b.left_confidence[0] == 0.0 and b.right_confidence[0] == 0.0


def test_boundary_error_of_a_shifted_prediction(oval, cam):
    gt = L.render_label(oval, _pose(oval, 10.0), cam, band_m=0.0, yaw_sigma=0.0)
    # A "prediction" that believes the track is 0.3 m further left.
    shifted = oval.shifted(0.0, 0.3)
    pred = L.render_label(shifted, _pose(oval, 10.0), cam, band_m=0.0, yaw_sigma=0.0)
    acc = BoundaryAccumulator(cam)
    acc.add(pred, gt)
    acc.add(gt, gt)
    s = acc.summary()
    e15 = error_at_range(s, 15.0)
    assert e15 is not None and 0.1 < e15 < 0.2  # 0.3 m on one frame, 0 on the other
    assert s["frames"] == 2 and s["missed_rows"] == 0
    perfect = BoundaryAccumulator(cam)
    perfect.add(gt, gt)
    assert error_at_range(perfect.summary(), 10.0) == 0.0


def test_empty_prediction_counts_misses(oval, cam):
    gt = L.render_label(oval, _pose(oval, 10.0), cam, band_m=0.0, yaw_sigma=0.0)
    acc = BoundaryAccumulator(cam)
    acc.add(np.zeros_like(gt), gt)
    s = acc.summary()
    assert s["missed_rows"] > 100 and all(v is None for v in s["mean_abs_error_m"])
