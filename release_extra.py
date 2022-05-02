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

env_vars = env.get("ENV")
print(env_vars)

print("Firmware Name: %s" % firmware_name)
print("Firmware Version: %s" % firmware_version)
print("MQTT Test: %s" % env_vars["MQTT_TEST"])
print("MQTT Username: %s" % env_vars["MQTT_USERNAME"])

env.Append(
    BUILD_FLAGS=[
        "-DFW_VERSION=%s" % (firmware_version),
        "-DMQTT_USERNAME=%s" % (env_vars["MQTT_USERNAME"]),
        "-DMQTT_PASSWORD=%s" % (env_vars["MQTT_PASSWORD"])
    ]
)

env.Replace(
    PROGNAME="%s_%s_v%s" % (firmware_name, env_name, firmware_version)
)
