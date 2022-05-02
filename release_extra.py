Import("env")

import subprocess

config = env.GetProjectConfig()

# get the firmware name from the [firmware] section in platformio.ini
firmware_name = config.get("firmware", "name")

# get the env name for this build
env_name = env.subst("$PIOENV")

# query the current version via git tags (unannotated)
ret = subprocess.run(["git", "describe", "--tags"], stdout=subprocess.PIPE, text=True)
firmware_version = ret.stdout.strip()

mqtt_username = env.subst("MQTT_USERNAME")
mqtt_password = env.subst("MQTT_PASSWORD")

print("Firmware Name: %s" % firmware_name)
print("Firmware Version: %s" % firmware_version)
print("MQTT Username: %s" % mqtt_username)

env.Append(
    BUILD_FLAGS=["-DFW_VERSION=%s" % (firmware_version)]
)

env.Append(
    BUILD_FLAGS=["-DMQTT_USERNAME=%s" % (mqtt_username)]
)

env.Append(
    BUILD_FLAGS=["-DMQTT_PASSWORD=%s" % (mqtt_password)]
)

env.Replace(
    PROGNAME="%s_%s_v%s" % (firmware_name, env_name, firmware_version)
)
