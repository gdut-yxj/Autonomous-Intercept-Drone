# **DRL_UAV_NAV**

![airsim_env](D:\Unreal\Unreal Projects\Autonomous-Intercept-Drone\png\airsim_env.png)

settings.json

```json
{
    "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "Vehicles": {
        "UAV1": {
            "VehicleType": "SimpleFlight",
            "X": 0,
            "Y": 0,
            "Z": 0,
            "Yaw": 0
        }
    },
    "ViewMode": "",
    "ClockSpeed": 10,
    "SubWindows": [
        {
            "WindowID": 0,
            "CameraID": 0,
            "ImageType": 0,
            "Visible": true
        },
        {
            "WindowID": 1,
            "CameraID": 0,
            "ImageType": 3,
            "Visible": false
        },
        {
            "WindowID": 2,
            "CameraID": 0,
            "ImageType": 3,
            "Visible": true
        }
    ],
    "CameraDefaults": {
        "CaptureSettings": [
            {
                "ImageType": 3,
                "Width": 100,
                "Height": 80,
                "FOV_Degrees": 90,
                "AutoExposureSpeed": 100,
                "AutoExposureBias": 0,
                "AutoExposureMaxBrightness": 0.64,
                "AutoExposureMinBrightness": 0.03,
                "MotionBlurAmount": 0,
                "TargetGamma": 1.0,
                "ProjectionMode": "",
                "OrthoWidth": 5.12
            },
            {
                "ImageType": 0,
                "Width": 256,
                "Height": 144,
                "FOV_Degrees": 90,
                "AutoExposureSpeed": 100,
                "AutoExposureBias": 0,
                "AutoExposureMaxBrightness": 0.64,
                "AutoExposureMinBrightness": 0.03,
                "MotionBlurAmount": 0,
                "TargetGamma": 1.0,
                "ProjectionMode": "",
                "OrthoWidth": 5.12
            }
        ]
    }
}

/*
    "Sensors": {
                "LidarSensor1": {
                    "SensorType": 6,
                    "Enabled": true,
                    "NumberOfChannels": 8,
                    "RotationsPerSecond": 10,
                    "Range": 12,
                    "PointsPerSecond": 64000,
                    "X": 0,
                    "Y": 0,
                    "Z": -1,
                    "Roll": 0,
                    "Pitch": 0,
                    "Yaw": 0,
                    "VerticalFOVUpper": 10,
                    "VerticalFOVLower": -10,
                    "HorizontalFOVStart": -180,
                    "HorizontalFOVEnd": 180,
                    "DrawDebugPoints": true,
                    "DataFrame": "VehicleInertialFrame"
                }
            }
*/

```
