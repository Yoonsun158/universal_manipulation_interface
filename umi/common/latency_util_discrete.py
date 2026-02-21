import numpy as np
from scipy.interpolate import interp1d
import scipy.signal as ss

def regular_sample(x, t, t_samples):
    # Kind='nearest' can be better for discrete square waves to avoid smoothed edges
    # However, KIND='linear' works fine for cross-correlation.
    spline = interp1d(x=t, y=x, kind='nearest', bounds_error=False, fill_value=(x[0], x[-1]))
    result = spline(t_samples)
    return result

def get_latency(
        x_target, t_target, 
        x_actual, t_actual, 
        t_start=None, t_end=None,
        resample_dt=1/1000,
        force_positive=False
        ):
    """
    Get latency for discrete/square-wave signals by cross-correlation.
    """
    # Filter out nan values
    x_actual = np.array(x_actual)
    t_actual = np.array(t_actual)
    valid_mask = ~np.isnan(x_actual)
    x_actual = x_actual[valid_mask]
    t_actual = t_actual[valid_mask]

    assert len(x_target) == len(t_target), f"Target length mismatch: {len(x_target)} and {len(t_target)}"
    assert len(x_actual) == len(t_actual), f"Actual length mismatch: {len(x_actual)} and {len(t_actual)}"
    
    if len(x_actual) == 0:
        return 0.0, {'t_samples': [], 'x_target': [], 'x_actual': [], 'correlation': [], 'lags': []}

    if t_start is None:
        t_start = max(t_target[0], t_actual[0])
    if t_end is None:
        t_end = min(t_target[-1], t_actual[-1])
    
    n_samples = int((t_end - t_start) / resample_dt)
    if n_samples <= 0:
        # Fallback to defaults
        t_start = min(t_target[0], t_actual[0])
        t_end = max(t_target[-1], t_actual[-1])
        n_samples = int((t_end - t_start) / resample_dt)
        
    t_samples = np.arange(n_samples) * resample_dt + t_start
    target_samples = regular_sample(x_target, t_target, t_samples)
    actual_samples = regular_sample(x_actual, t_actual, t_samples)

    # Normalize samples to zero mean unit std
    # This ensures cross-correlation peak corresponds to best overlap
    all_vals = np.concatenate([target_samples, actual_samples])
    mean = np.mean(all_vals)
    std = np.std(all_vals)
    
    if std < 1e-6:
        # Avoid division by zero if signal is constant
        target_norm = target_samples - mean
        actual_norm = actual_samples - mean
    else:
        target_norm = (target_samples - mean) / std
        actual_norm = (actual_samples - mean) / std

    # Cross-correlation
    correlation = ss.correlate(actual_norm, target_norm, mode='full')
    lags = ss.correlation_lags(len(actual_norm), len(target_norm))
    t_lags = lags * resample_dt

    if force_positive:
        positive_mask = t_lags >= 0
        if np.any(positive_mask):
            latency = t_lags[positive_mask][np.argmax(correlation[positive_mask])]
        else:
            latency = t_lags[np.argmax(correlation)]
    else:
        latency = t_lags[np.argmax(correlation)]
    
    info = {
        't_samples': t_samples,
        'x_target': target_norm,
        'x_actual': actual_norm,
        'correlation': correlation,
        'lags': t_lags
    }

    return latency, info
