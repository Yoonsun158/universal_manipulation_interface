import numpy as np
from scipy.interpolate import interp1d
import scipy.signal as ss

def regular_sample(x, t, t_samples):
    spline = interp1d(x=t, y=x, bounds_error=False, fill_value=(x[0], x[-1]))
    result = spline(t_samples)
    return result

def get_latency(
        x_target, t_target, 
        x_actual, t_actual, 
        # t_start=None, t_end=None,
        t_start_target=None,
        t_start_actual=None,
        duration_target=None,
        duration_actual=None,
        resample_dt=1/1000,
        force_positive=False
        ):
    assert len(x_target) == len(t_target)
    assert len(x_actual) == len(t_actual)
    if t_start_target is None:
        t_start_target = t_target[0]
    if t_start_actual is None:
        t_start_actual = t_actual[0]
    if duration_target is None:
        duration_target = t_target[-1] - t_target[0]
    if duration_actual is None:
        duration_actual = t_actual[-1] - t_actual[0]
    n_samples_target = int(duration_target / resample_dt)
    n_samples_actual = int(duration_actual / resample_dt)
    t_samples_target = np.arange(n_samples_target) * resample_dt + t_start_target
    t_samples_actual = np.arange(n_samples_actual) * resample_dt + t_start_actual

    target_samples = regular_sample(x_target, t_target, t_samples_target)
    actual_samples = regular_sample(x_actual, t_actual, t_samples_actual)

    # normalize samples to zero mean unit std
    mean = np.mean(np.concatenate([target_samples, actual_samples]))
    std = np.std(np.concatenate([target_samples, actual_samples]))
    target_samples = (target_samples - mean) / std
    actual_samples = (actual_samples - mean) / std

    # cross correlation
    correlation = ss.correlate(actual_samples, target_samples)
    lags = ss.correlation_lags(len(actual_samples), len(target_samples))
    t_lags = lags * resample_dt

    latency = None
    if force_positive:
        latency = t_lags[np.argmax(correlation[t_lags >= 0])]
    else:
        latency = t_lags[np.argmax(correlation)]
    
    info = {
        't_samples': t_samples_target,
        'x_target': target_samples,
        'x_actual': actual_samples,
        'correlation': correlation,
        'lags': t_lags
    }

    return latency, info
