import lgpio
import time
import os
import glob

# Motor speeds: -MAX_SPEED to MAX_SPEED (480 for historical compatibility)
_max_speed = 480
MAX_SPEED = _max_speed

_pin_M1FLT = 5
_pin_M2FLT = 6
_pin_M1EN = 22
_pin_M2EN = 23
_pin_M1DIR = 24
_pin_M2DIR = 25

# PWM is driven by the RP1 *hardware* PWM, not lgpio software PWM, so we can run
# ultrasonic (24 kHz) and kill the audible motor whine. The Pololu Dual G2 HPMD
# accepts PWM up to 100 kHz, so the driver is not the limit.
#   GPIO12 -> PWM channel 0 (M1),  GPIO13 -> PWM channel 1 (M2)
# Requires in /boot/firmware/config.txt (then a reboot):
#   dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
_PWM_FREQ = 24000  # Hz (ultrasonic)
_PWM_PERIOD_NS = int(1e9 / _PWM_FREQ)
_M1_PWM_CHAN = 0  # GPIO12
_M2_PWM_CHAN = 1  # GPIO13

# Pi 5 RP1 is /dev/gpiochip0 (gpiochip4 is a symlink that Docker can't resolve)
_h = lgpio.gpiochip_open(0)
if _h < 0:
    raise IOError("Can't open gpiochip0 — is /dev/gpiochip0 accessible?")


def _find_pwmchip():
    """Locate the sysfs PWM chip created by the pwm-2chan overlay.

    Override with MOTOR_PWMCHIP=pwmchipN if numbering ever collides with
    another PWM (e.g. a fan controller). Otherwise pick the first chip with
    >=2 channels — on this Pi there are none until the overlay loads, so the
    overlay's chip is the only candidate and its number can change freely.
    """
    override = os.environ.get("MOTOR_PWMCHIP")
    if override:
        return "/sys/class/pwm/" + override
    for chip in sorted(glob.glob("/sys/class/pwm/pwmchip*")):
        try:
            with open(os.path.join(chip, "npwm")) as f:
                if int(f.read()) >= 2:
                    return chip
        except OSError:
            continue
    raise IOError(
        "No PWM chip with >=2 channels in /sys/class/pwm. Add "
        "'dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4' to "
        "/boot/firmware/config.txt and reboot.")


class _HwPwm(object):
    """One hardware-PWM channel via the sysfs interface, fixed period."""

    def __init__(self, chip, channel, period_ns):
        self.path = os.path.join(chip, "pwm%d" % channel)
        if not os.path.isdir(self.path):
            with open(os.path.join(chip, "export"), "w") as f:
                f.write(str(channel))
            # udev needs a moment to create the node and grant write perms
            for _ in range(100):
                if os.access(os.path.join(self.path, "duty_cycle"), os.W_OK):
                    break
                time.sleep(0.01)
        self._try_write("enable", 0)        # polarity is only writable when off
        self._write("duty_cycle", 0)        # always valid (0 <= any period)
        self._write("period", period_ns)
        self._try_write("polarity", "normal")
        self._write("enable", 1)

    def _write(self, attr, value):
        with open(os.path.join(self.path, attr), "w") as f:
            f.write(str(value))

    def _try_write(self, attr, value):
        try:
            self._write(attr, value)
        except OSError:
            pass

    def setDutyFraction(self, frac):
        frac = 0.0 if frac < 0 else (1.0 if frac > 1 else frac)
        self._write("duty_cycle", int(_PWM_PERIOD_NS * frac))


_PWMCHIP = _find_pwmchip()


class Motor(object):
    MAX_SPEED = _max_speed

    def __init__(self, pwm_chan, dir_pin, en_pin, flt_pin):
        self.dir_pin = dir_pin
        self.en_pin = en_pin
        self.flt_pin = flt_pin

        lgpio.gpio_claim_input(_h, flt_pin, lgpio.SET_PULL_UP)
        lgpio.gpio_claim_output(_h, dir_pin)
        lgpio.gpio_claim_output(_h, en_pin)
        lgpio.gpio_write(_h, en_pin, 1)
        time.sleep(0.05)  # let FLT pull-up settle before first fault read

        # GPIO12/13 are owned by the kernel PWM (overlay), not lgpio.
        self.pwm = _HwPwm(_PWMCHIP, pwm_chan, _PWM_PERIOD_NS)

    def setSpeed(self, speed):
        if speed < 0:
            speed = -speed
            dir_value = 1
        else:
            dir_value = 0

        if speed > MAX_SPEED:
            speed = MAX_SPEED

        lgpio.gpio_write(_h, self.dir_pin, dir_value)
        self.pwm.setDutyFraction(speed / MAX_SPEED)

    def enable(self):
        lgpio.gpio_write(_h, self.en_pin, 1)

    def disable(self):
        lgpio.gpio_write(_h, self.en_pin, 0)

    def getFault(self):
        return not lgpio.gpio_read(_h, self.flt_pin)


class Motors(object):
    MAX_SPEED = _max_speed

    def __init__(self):
        self.motor1 = Motor(_M1_PWM_CHAN, _pin_M1DIR, _pin_M1EN, _pin_M1FLT)
        self.motor2 = Motor(_M2_PWM_CHAN, _pin_M2DIR, _pin_M2EN, _pin_M2FLT)

    def setSpeeds(self, m1_speed, m2_speed):
        self.motor1.setSpeed(-m1_speed)
        self.motor2.setSpeed(m2_speed)

    def enable(self):
        self.motor1.enable()
        self.motor2.enable()

    def disable(self):
        self.motor1.disable()
        self.motor2.disable()

    def getFaults(self):
        return self.motor1.getFault() or self.motor2.getFault()

    def forceStop(self):
        self.setSpeeds(0, 0)


motors = Motors()