#!/usr/bin/env python3
"""Top-yaw sweep coupling analysis.

Analyzes CSV logs produced by rmcs_core::controller::identification::
TopYawSweepController (default /tmp/top_yaw_sweep_*.csv) and estimates the
coupling from the top-yaw sweep motion into the roll angle closed loop.

Outputs (Bode CSV + PNG figures):
    bode.csv
    01_timeseries.png
    02_bode_angle_to_roll_error.png
    03_bode_accel_to_roll_error.png
    04_bode_angle_to_roll_torque.png
    05_coherence.png
    06_roll_error_spectrum.png

Dependencies:
    numpy, matplotlib
    sudo apt-get install -y python3-matplotlib
    # or: python3 -m pip install matplotlib
"""

import argparse
import glob
import os
import sys

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as exception:  # pragma: no cover - dependency hint
    sys.exit(
        "matplotlib is required for plotting: {}\n"
        "Install it with: sudo apt-get install -y python3-matplotlib".format(exception)
    )


REQUIRED_COLUMNS = (
    "elapsed_s",
    "freq_hz",
    "ref_angle",
    "top_yaw_angle",
    "top_yaw_velocity",
    "top_yaw_torque",
    "test_torque",
    "gimbal_torque",
    "roll_angle",
    "roll_velocity",
    "roll_torque",
    "roll_control_torque",
)


def wrap_to_pi(angle):
    return np.remainder(angle + np.pi, 2.0 * np.pi) - np.pi


def moving_average(values, window):
    if window <= 1:
        return values
    kernel = np.ones(window) / window
    return np.convolve(values, kernel, mode="same")


class Run:
    def __init__(self, path, roll_ref):
        data = np.genfromtxt(path, delimiter=",", names=True)
        if data.size == 0:
            raise ValueError("empty csv: {}".format(path))

        missing = [name for name in REQUIRED_COLUMNS if name not in data.dtype.names]
        if missing:
            raise ValueError("{} missing columns: {}".format(path, ", ".join(missing)))

        self.path = path
        self.t = np.asarray(data["elapsed_s"], dtype=float)
        self.freq = np.asarray(data["freq_hz"], dtype=float)
        self.ref = np.asarray(data["ref_angle"], dtype=float)
        self.angle = np.asarray(data["top_yaw_angle"], dtype=float)
        self.velocity = np.asarray(data["top_yaw_velocity"], dtype=float)
        self.top_yaw_torque = np.asarray(data["top_yaw_torque"], dtype=float)
        self.test_torque = np.asarray(data["test_torque"], dtype=float)
        self.gimbal_torque = np.asarray(data["gimbal_torque"], dtype=float)
        self.roll_angle = np.asarray(data["roll_angle"], dtype=float)
        self.roll_velocity = np.asarray(data["roll_velocity"], dtype=float)
        self.roll_torque = np.asarray(data["roll_torque"], dtype=float)
        self.roll_control_torque = np.asarray(data["roll_control_torque"], dtype=float)

        self.roll_error = wrap_to_pi(roll_ref - self.roll_angle)

        if self.t.size >= 3:
            dt = np.median(np.diff(self.t))
            velocity = np.gradient(self.angle, self.t)
            self.acceleration = moving_average(np.gradient(velocity, self.t), 5)
            self.dt = dt
        else:
            self.acceleration = np.zeros_like(self.angle)
            self.dt = 0.0

        # Swept-sine phase obtained by integrating the logged instantaneous
        # frequency, so demodulation follows the chirp instead of a fixed tone.
        increments = 0.5 * (self.freq[1:] + self.freq[:-1]) * np.diff(self.t)
        self.phase = 2.0 * np.pi * np.concatenate(([0.0], np.cumsum(increments)))


def window_for_frequency(t, freq, frequency_hz, cycles):
    """Select the time window centered where the chirp frequency crosses frequency_hz."""
    center_index = int(np.argmin(np.abs(freq - frequency_hz)))
    t_center = t[center_index]
    half_window = 0.5 * cycles / frequency_hz
    mask = (t >= t_center - half_window) & (t <= t_center + half_window)
    return mask, t_center


def amplitude_at(t, y, freq, phase, frequency_hz, cycles):
    """Peak amplitude of y demodulated against the chirp phase at one frequency."""
    mask, _ = window_for_frequency(t, freq, frequency_hz, cycles)
    count = int(np.count_nonzero(mask))
    if count < 16:
        return np.nan

    yy = y[mask] - np.mean(y[mask])
    window = np.hanning(count)
    window_sum = np.sum(window)
    if window_sum <= 0.0:
        return np.nan

    # For a real sinusoid the demodulated spectrum is A / 2 * sum(window).
    return 2.0 * abs(np.sum(window * yy * np.exp(-1j * phase[mask]))) / window_sum


def frf_at(t, u, y, freq, phase, frequency_hz, cycles, subwindows=4):
    """Least-squares / I-Q demodulation of H = Y / U at one frequency."""
    mask, t_center = window_for_frequency(t, freq, frequency_hz, cycles)
    count = int(np.count_nonzero(mask))
    if count < 16:
        return np.nan, np.nan, t_center

    uu = u[mask] - np.mean(u[mask])
    yy = y[mask] - np.mean(y[mask])
    window = np.hanning(count)
    demodulation = np.exp(-1j * phase[mask])
    u_spectrum = np.sum(window * uu * demodulation)
    y_spectrum = np.sum(window * yy * demodulation)
    transfer = y_spectrum / u_spectrum if abs(u_spectrum) > 1e-12 else np.nan

    edges = np.linspace(0, count, subwindows + 1).astype(int)
    u_subs = []
    y_subs = []
    for start, stop in zip(edges[:-1], edges[1:]):
        if stop - start < 4:
            continue
        sub_window = np.hanning(stop - start)
        sub_phase = demodulation[start:stop]
        sub_u = uu[start:stop] - np.mean(uu[start:stop])
        sub_y = yy[start:stop] - np.mean(yy[start:stop])
        u_subs.append(np.sum(sub_window * sub_u * sub_phase))
        y_subs.append(np.sum(sub_window * sub_y * sub_phase))

    if len(u_subs) >= 2:
        u_subs = np.asarray(u_subs)
        y_subs = np.asarray(y_subs)
        denominator = np.sum(np.abs(u_subs) ** 2) * np.sum(np.abs(y_subs) ** 2)
        coherence = (
            abs(np.sum(np.conj(u_subs) * y_subs)) ** 2 / denominator
            if denominator > 0.0
            else np.nan
        )
    else:
        coherence = np.nan

    return transfer, coherence, t_center


def analyze_run(run, frequencies, cycles):
    results = {}
    for name, u, y in (
        ("angle_to_roll_error", run.angle, run.roll_error),
        ("accel_to_roll_error", run.acceleration, run.roll_error),
        ("angle_to_roll_torque", run.angle, run.roll_control_torque),
    ):
        transfer = np.full(frequencies.shape, np.nan, dtype=complex)
        coherence = np.full(frequencies.shape, np.nan, dtype=float)
        for index, frequency_hz in enumerate(frequencies):
            transfer[index], coherence[index], _ = frf_at(
                run.t, u, y, run.freq, run.phase, frequency_hz, cycles
            )
        results[name] = (transfer, coherence)

    amplitudes = {}
    for name, y in (
        ("roll_error", run.roll_error),
        ("top_yaw_angle", run.angle),
        ("roll_control_torque", run.roll_control_torque),
    ):
        amplitudes[name] = np.array(
            [
                amplitude_at(run.t, y, run.freq, run.phase, frequency_hz, cycles)
                for frequency_hz in frequencies
            ]
        )

    return results, amplitudes


def complex_nanmean(values, axis=0):
    return np.nanmean(np.real(values), axis=axis) + 1j * np.nanmean(np.imag(values), axis=axis)


def plot_timeseries(run, output_path):
    figure, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    axes[0].plot(run.t, run.ref, label="top_yaw ref [rad]", linewidth=1.0)
    axes[0].plot(run.t, run.angle, label="top_yaw angle [rad]", linewidth=1.0)
    axes[0].set_ylabel("top yaw [rad]")
    axes[0].legend(loc="upper right")
    frequency_axis = axes[0].twinx()
    frequency_axis.plot(run.t, run.freq, color="0.6", linewidth=1.0, label="freq [Hz]")
    frequency_axis.set_ylabel("freq [Hz]")

    axes[1].plot(run.t, np.degrees(run.roll_error), color="tab:red", linewidth=1.0)
    axes[1].set_ylabel("roll error [deg]")

    axes[2].plot(run.t, run.test_torque, label="test torque [N*m]", linewidth=1.0)
    axes[2].plot(run.t, run.gimbal_torque, label="gimbal torque [N*m]", linewidth=1.0)
    axes[2].plot(run.t, run.roll_control_torque, label="roll control [N*m]", linewidth=1.0)
    axes[2].set_ylabel("torque [N*m]")
    axes[2].set_xlabel("elapsed [s]")
    axes[2].legend(loc="upper right")

    figure.suptitle(os.path.basename(run.path))
    figure.tight_layout()
    figure.savefig(output_path, dpi=150)
    plt.close(figure)


def plot_bode(frequencies, transfer, coherence, title, output_path):
    magnitude_db = 20.0 * np.log10(np.maximum(np.abs(transfer), 1e-12))
    phase_deg = np.degrees(np.unwrap(np.angle(transfer)))

    figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].semilogx(frequencies, magnitude_db, marker="o", markersize=3)
    axes[0].set_ylabel("magnitude [dB]")
    axes[0].grid(True, which="both", alpha=0.3)
    mean_coherence = np.nanmean(coherence) if np.any(np.isfinite(coherence)) else np.nan
    axes[0].set_title("{} (mean coherence {:.2f})".format(title, mean_coherence))

    axes[1].semilogx(frequencies, phase_deg, marker="o", markersize=3, color="tab:orange")
    axes[1].set_ylabel("phase [deg]")
    axes[1].set_xlabel("frequency [Hz]")
    axes[1].grid(True, which="both", alpha=0.3)

    figure.tight_layout()
    figure.savefig(output_path, dpi=150)
    plt.close(figure)

    return magnitude_db, phase_deg


def plot_coherence(frequencies, curves, output_path):
    figure, axis = plt.subplots(figsize=(10, 4))
    for label, coherence in curves.items():
        axis.semilogx(frequencies, coherence, marker="o", markersize=3, label=label)
    axis.set_xlabel("frequency [Hz]")
    axis.set_ylabel("coherence")
    axis.set_ylim(0.0, 1.05)
    axis.grid(True, which="both", alpha=0.3)
    axis.legend(loc="lower left")
    figure.tight_layout()
    figure.savefig(output_path, dpi=150)
    plt.close(figure)


def plot_roll_error_spectrum(
    frequencies, roll_error_deg, top_yaw_deg, reference_deg, output_path
):
    figure, axis = plt.subplots(figsize=(10, 5))
    axis.semilogx(
        frequencies,
        roll_error_deg,
        marker="o",
        markersize=3,
        color="tab:red",
        label="roll error amplitude",
    )
    axis.axhline(
        reference_deg,
        color="0.6",
        linestyle="--",
        linewidth=1.0,
        label="top_yaw reference amplitude",
    )
    axis.set_xlabel("frequency [Hz]")
    axis.set_ylabel("roll error amplitude [deg]")
    axis.grid(True, which="both", alpha=0.3)

    if np.any(np.isfinite(roll_error_deg)):
        peak_index = int(np.nanargmax(roll_error_deg))
        axis.annotate(
            "{:.3f} Hz, {:.3f} deg".format(frequencies[peak_index], roll_error_deg[peak_index]),
            xy=(frequencies[peak_index], roll_error_deg[peak_index]),
            xytext=(8, 8),
            textcoords="offset points",
        )

    secondary = axis.twinx()
    secondary.semilogx(
        frequencies,
        top_yaw_deg,
        marker="s",
        markersize=3,
        color="tab:blue",
        alpha=0.6,
        label="top_yaw angle amplitude",
    )
    secondary.set_ylabel("top_yaw angle amplitude [deg]")

    lines = axis.get_legend_handles_labels()[0] + secondary.get_legend_handles_labels()[0]
    labels = axis.get_legend_handles_labels()[1] + secondary.get_legend_handles_labels()[1]
    axis.legend(lines, labels, loc="upper left")

    figure.tight_layout()
    figure.savefig(output_path, dpi=150)
    plt.close(figure)

    return roll_error_deg


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", help="sweep csv file(s) or glob pattern(s)")
    parser.add_argument("--roll-ref", type=float, default=2.948, help="roll angle setpoint [rad]")
    parser.add_argument("--output-dir", default=None, help="report directory")
    parser.add_argument("--start-freq", type=float, default=None, help="analysis low frequency [Hz]")
    parser.add_argument("--end-freq", type=float, default=None, help="analysis high frequency [Hz]")
    parser.add_argument("--points", type=int, default=40, help="number of log-spaced frequencies")
    parser.add_argument("--cycles", type=float, default=6.0, help="window length in cycles")
    return parser.parse_args()


def main():
    args = parse_args()

    paths = []
    for pattern in args.csv:
        matched = sorted(glob.glob(pattern))
        paths.extend(matched if matched else [pattern])
    paths = [path for path in paths if os.path.isfile(path)]
    if not paths:
        sys.exit("no csv file found")

    runs = []
    for path in paths:
        try:
            runs.append(Run(path, args.roll_ref))
        except ValueError as exception:
            sys.exit(str(exception))

    frequency_min = args.start_freq
    frequency_max = args.end_freq
    if frequency_min is None:
        frequency_min = max(1e-3, min(np.nanmin(run.freq[run.freq > 0.0]) for run in runs))
    if frequency_max is None:
        frequency_max = max(np.nanmax(run.freq) for run in runs)
    if frequency_min <= 0.0 or frequency_max <= frequency_min:
        sys.exit("invalid analysis frequency range")

    frequencies = np.logspace(np.log10(frequency_min), np.log10(frequency_max), args.points)

    per_run = [analyze_run(run, frequencies, args.cycles) for run in runs]
    keys = ("angle_to_roll_error", "accel_to_roll_error", "angle_to_roll_torque")
    merged = {}
    for key in keys:
        transfers = np.vstack([results[key][0] for results, _ in per_run])
        coherences = np.vstack([results[key][1] for results, _ in per_run])
        merged[key] = (complex_nanmean(transfers), np.nanmean(coherences, axis=0))

    amplitude_curves = {}
    for name in ("roll_error", "top_yaw_angle", "roll_control_torque"):
        stacked = np.vstack([amplitudes[name] for _, amplitudes in per_run])
        amplitude_curves[name] = np.nanmean(stacked, axis=0)

    roll_error_deg = np.degrees(amplitude_curves["roll_error"])
    top_yaw_deg = np.degrees(amplitude_curves["top_yaw_angle"])
    roll_control_amp = amplitude_curves["roll_control_torque"]
    reference_deg = np.degrees(0.5 * (np.nanmax(runs[0].ref) - np.nanmin(runs[0].ref)))

    output_dir = args.output_dir
    if output_dir is None:
        output_dir = os.path.join(os.path.dirname(os.path.abspath(paths[0])), "top_yaw_sweep_report")
    os.makedirs(output_dir, exist_ok=True)

    h1, c1 = merged["angle_to_roll_error"]
    h2, c2 = merged["accel_to_roll_error"]
    h3, c3 = merged["angle_to_roll_torque"]

    mag1, phase1 = plot_bode(
        frequencies, h1, c1, "H1: top_yaw angle -> roll error", os.path.join(output_dir, "02_bode_angle_to_roll_error.png")
    )
    mag2, phase2 = plot_bode(
        frequencies, h2, c2, "H2: top_yaw accel -> roll error", os.path.join(output_dir, "03_bode_accel_to_roll_error.png")
    )
    mag3, phase3 = plot_bode(
        frequencies, h3, c3, "H3: top_yaw angle -> roll control torque", os.path.join(output_dir, "04_bode_angle_to_roll_torque.png")
    )
    plot_coherence(
        frequencies,
        {
            "angle -> roll error": c1,
            "accel -> roll error": c2,
            "angle -> roll torque": c3,
        },
        os.path.join(output_dir, "05_coherence.png"),
    )
    plot_timeseries(runs[0], os.path.join(output_dir, "01_timeseries.png"))
    plot_roll_error_spectrum(
        frequencies,
        roll_error_deg,
        top_yaw_deg,
        reference_deg,
        os.path.join(output_dir, "06_roll_error_spectrum.png"),
    )

    bode_path = os.path.join(output_dir, "bode.csv")
    with open(bode_path, "w") as file:
        file.write(
            "freq_hz,H1_mag_db,H1_phase_deg,H1_coh,H2_mag_db,H2_phase_deg,H2_coh,"
            "H3_mag_db,H3_phase_deg,H3_coh,roll_error_amp_deg,top_yaw_amp_deg,roll_control_amp\n"
        )
        for index, frequency_hz in enumerate(frequencies):
            file.write(
                "{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},"
                "{:.6f},{:.6f},{:.6f}\n".format(
                    frequency_hz,
                    mag1[index],
                    phase1[index],
                    c1[index],
                    mag2[index],
                    phase2[index],
                    c2[index],
                    mag3[index],
                    phase3[index],
                    c3[index],
                    roll_error_deg[index],
                    top_yaw_deg[index],
                    roll_control_amp[index],
                )
            )

    roll_error_rms = np.sqrt(np.nanmean(np.square(np.degrees(runs[0].roll_error))))
    valid = np.isfinite(np.abs(h2))
    peak_text = "n/a"
    if np.any(valid):
        peak_index = int(np.nanargmax(np.abs(h2)))
        peak_text = "{:.3f} Hz".format(frequencies[peak_index])

    roll_peak_text = "n/a"
    if np.any(np.isfinite(roll_error_deg)):
        roll_peak_index = int(np.nanargmax(roll_error_deg))
        roll_peak_text = "{:.3f} Hz ({:.4f} deg)".format(
            frequencies[roll_peak_index], roll_error_deg[roll_peak_index]
        )

    print("runs analyzed      : {}".format(len(runs)))
    print("frequency range    : {:.3f} ~ {:.3f} Hz".format(frequency_min, frequency_max))
    print("roll error RMS     : {:.3f} deg".format(roll_error_rms))
    print("peak roll error    : {}".format(roll_peak_text))
    print("peak |H2| (accel)  : {}".format(peak_text))
    print("report directory   : {}".format(output_dir))


if __name__ == "__main__":
    main()
