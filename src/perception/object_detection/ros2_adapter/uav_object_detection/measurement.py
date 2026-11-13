# SPDX-License-Identifier: GPL-3.0-or-later
"""Pinned stage2 depth projection; no target association, KF or prediction."""
from collections import deque
import numpy as np


class Samples:
    def __init__(self, limit=60):
        self.items = deque(maxlen=limit)

    def add(self, stamp, value):
        if not np.isfinite(stamp) or stamp <= 0 or (self.items and stamp <= self.items[-1][0]):
            return False
        self.items.append((stamp, value))
        return True

    def nearest(self, stamp, tolerance):
        return self.match(stamp, tolerance)[1]

    def match(self, stamp, tolerance):
        if not self.items:
            raise ValueError('missing synchronized input')
        t, value = min(self.items, key=lambda item: abs(item[0]-stamp))
        if abs(t-stamp) > tolerance:
            raise ValueError('synchronization tolerance exceeded')
        return t, value


def locate(depth, box, intrinsics, body_rotation, body_position, extrinsic, config, diagnostics=None, mask=None):
    """Apply the pinned upstream Detect depth algorithm, then ROS/world transforms."""
    from . import gemini2_depth as reference
    d = diagnostics if diagnostics is not None else {}
    d.update(method='gemini2_reference', box=list(box), accepted=False)
    def reject(reason):
        d['reason'] = reason
        raise ValueError(reason)
    if config.get('depth_method','gemini2_reference') != 'gemini2_reference':
        reject('unsupported depth method')
    if depth.ndim != 2 or not np.isfinite(intrinsics).all():
        reject('invalid calibration/depth')
    fx, fy, cx, cy = intrinsics
    if min(fx, fy) <= 0:
        reject('invalid focal length')
    if len(box) != 4 or not np.isfinite(box).all():
        reject('invalid detection box')
    # Same int32 truncation/clipping as upstream process_yolo_result().
    x1,y1,x2,y2 = np.asarray(box).astype(np.int32).tolist()
    height,width = depth.shape
    if not height or not width:
        reject('invalid calibration/depth')
    x1=max(0,min(x1,width-1)); y1=max(0,min(y1,height-1))
    x2=max(x1+1,min(x2,width)); y2=max(y1+1,min(y2,height))
    region = reference.create_center_roi_mask(depth.shape,(x1,y1,x2,y2))
    d['depth_source']='center_roi'
    if mask is not None:
        if mask.shape!=depth.shape or not np.isin(mask,[0,1]).all():
            reject('invalid instance mask geometry/values')
        prepared_mask=reference.prepare_segmentation_mask(mask,depth.shape)
        if np.count_nonzero(prepared_mask)>=reference.MIN_VALID_DEPTH_POINTS:
            region=prepared_mask
            d['depth_source']='seg_mask'
    # Bridge inputs are planar Z in meters; upstream computes in millimeters.
    millimeters = np.asarray(depth*1000.,dtype=np.float32)
    valid = region & (millimeters>reference.MIN_DEPTH_MM) & (millimeters<reference.MAX_DEPTH_MM)
    yy,xx = np.nonzero(region)
    d.update(roi=[int(xx.min()),int(yy.min()),int(xx.max()+1),int(yy.max()+1)],
             pixels=int(region.sum()),valid_pixels=int(valid.sum()),
             valid_ratio=float(valid.sum()/region.sum()),roi_ratio=reference.CENTER_ROI_RATIO,
             min_depth_m=reference.MIN_DEPTH_MM/1000,max_depth_m=reference.MAX_DEPTH_MM/1000)
    if valid.any():
        d['percentiles_m']=np.percentile(millimeters[valid]/1000.,[0,10,50,90,100]).tolist()
    estimate = reference.estimate_depth_from_region(millimeters,region)
    if estimate is None:
        reject('insufficient valid depth points (<20)')
    u,v,z = estimate.u,estimate.v,estimate.depth_mm/1000.
    # Diagnostic spread only: no added spread/center-support acceptance gate.
    values=millimeters[valid]
    bins,counts=np.unique(np.floor(values/reference.DEPTH_HIST_BIN_MM).astype(np.int32),return_counts=True)
    center=(bins[np.argmax(counts)]+.5)*reference.DEPTH_HIST_BIN_MM
    selected=values[np.abs(values-center)<=reference.DEPTH_CLUSTER_WINDOW_MM]
    fallback=selected.size<reference.MIN_VALID_DEPTH_POINTS
    if fallback:selected=values
    spread=float(np.percentile(selected,90)-np.percentile(selected,10))/1000.
    camera=np.array([(u-cx)*z/fx,(v-cy)*z/fy,z])
    local=body_rotation @ (extrinsic[0] @ camera + extrinsic[1]) + body_position
    if not np.isfinite(local).all():
        reject('nonfinite projection')
    d.update(accepted=True,reason='accepted',depth_m=z,representative_pixel=[u,v],
             cluster_pixels=estimate.cluster_points,fallback_to_all=fallback,spread_m=spread)
    # Existing KF adapter needs a variance; this is not an upstream validity gate.
    return local,max(.03,spread)**2


