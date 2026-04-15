import numpy as np

battery_open_voltage = 12.961
brownout_voltage = 6.75

# totals on pdh
current_to_voltage = {
    2: 12.28,
    6: 12.18,
    70: 10,
    98: 10.6,
    146: 9.53
}

x = list(current_to_voltage.keys())
y = [battery_open_voltage - v for v in current_to_voltage.values()]

resistance, _ = np.polyfit(x, y, 1)

print("Resistance between battery and pdh: " + str(resistance))

max_current = (battery_open_voltage - brownout_voltage) / resistance

# not a super great approx, varies with battery voltage loss under a current load for the battery,
# probably closer to max_current on the energizer battery
lasting_current_scaler = 0.8
lasting_current = lasting_current_scaler * max_current

print("Max current: " + str(max_current))
print("Lasting current: " + str(lasting_current))

# worst-case actions
drive_and_intake_feed = 40 * 4 + 25 * 4 + 40
windup = 50 * 2
shooting = 50 * 2 + 45 + 45

worst_case_actions = {
    "drive and intake feed": drive_and_intake_feed,
    "windup": windup,
    "shooting": shooting
}

print()

for name, current in worst_case_actions.items():
    if current > lasting_current:
        print("Warning action: " + name)
        print(current)
        print()
