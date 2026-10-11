"""Camera lane geometry, independent of ROS and the vehicle controller."""
import math
import numpy as np


def rotation(roll, pitch):
    c, s = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    return np.array([[cp, sp*s, sp*c], [0, c, -s], [-sp, cp*s, cp*c]])


def ground_points(pixels, width, height, hfov, camera_height, pitch_down,
                  roll=0., pitch=0., camera_x=0.):
    f = width / (2 * math.tan(math.radians(hfov) / 2))
    uv = np.asarray(pixels, dtype=float)
    rays = np.column_stack((np.ones(len(uv)), -(uv[:, 0]-width/2)/f,
                            -(uv[:, 1]-height/2)/f))
    rays = rays @ (rotation(roll, pitch) @ rotation(0, pitch_down)).T
    distance = np.full(len(rays), np.nan)
    np.divide(-camera_height, rays[:, 2], out=distance, where=rays[:, 2] < -1e-4)
    points = rays * distance[:, None]
    points[:, 0] += camera_x
    return points[:, :2]


def fit_heading(left, right, preview=4., min_width=2.5, max_width=4.8,
                max_rms=.12):
    """Require two boundaries; no assumed lane width or lateral-position update."""
    fits = []
    for points in (left, right):
        q = np.asarray(points)
        q = q[np.isfinite(q).all(axis=1) & (q[:, 0] > 2.) & (q[:, 0] < 15.)]
        if len(q) < 6 or np.ptp(q[:, 0]) < 3. or not q[:, 0].min() <= preview <= q[:, 0].max():
            return None
        fit = np.polyfit(q[:, 0], q[:, 1], 2)
        rms = np.sqrt(np.mean((np.polyval(fit, q[:, 0])-q[:, 1])**2))
        if rms > max_rms:
            return None
        fits.append((fit, rms))
    a, b = fits[0][0], fits[1][0]
    lane_width = float(np.polyval(a-b, preview))
    if not min_width <= lane_width <= max_width:
        return None
    center = (a+b)/2
    angle = math.atan(float(np.polyval(np.polyder(center), preview)))
    if abs(angle) > math.radians(18):
        return None
    return angle, lane_width, max(fits[0][1], fits[1][1])


def path_tangent(points, x, y, preview):
    """Project on path segments then advance by metric arc length."""
    p = np.asarray(points, dtype=float)
    delta = np.diff(p, axis=0)
    length = np.linalg.norm(delta, axis=1)
    valid = length > 1e-5
    if not valid.any():
        return None
    t = np.clip(np.sum((np.array([x, y])-p[:-1])*delta, axis=1)
                / np.maximum(length**2, 1e-10), 0., 1.)
    foot = p[:-1]+delta*t[:, None]
    dist = np.linalg.norm(foot-[x, y], axis=1)
    dist[~valid] = np.inf
    i = int(np.argmin(dist))
    remaining = preview + t[i]*length[i]
    while i < len(length)-1 and remaining > length[i]:
        remaining -= length[i]
        i += 1
    if not valid[i] or remaining > length[i]:
        return None
    return math.atan2(delta[i, 1], delta[i, 0]), float(np.min(dist))


def sample_rows(rows, stamp, angle_indices=(), max_extrapolation=.04):
    """Interpolate, or extrapolate at most 40ms from two fresh sensor rows."""
    import bisect
    if len(rows)<2: return None
    i=bisect.bisect_left([r[0] for r in rows],stamp)
    if i<len(rows) and abs(rows[i][0]-stamp)<1e-6: return list(rows[i][1:])
    if i==0:
        if rows[0][0]-stamp>max_extrapolation:return None
        a,b=rows[0],rows[1]
    elif i==len(rows):
        if stamp-rows[-1][0]>max_extrapolation:return None
        a,b=rows[-2],rows[-1]
    else:a,b=rows[i-1],rows[i]
    dt=b[0]-a[0]
    if not 0<dt<=.16:return None
    u=(stamp-a[0])/dt;out=[]
    for j in range(1,len(a)):
        d=b[j]-a[j]
        if j in angle_indices:d=math.atan2(math.sin(d),math.cos(d))
        out.append(a[j]+u*d)
    return out if np.isfinite(out).all() else None
