# jsk teaching object package

This package provides a simple human teachable function for the robot to recognize objects.


## Training

comming soon.

## Run trained models.

```
roslaunch jsk_teaching_object edgetpu_detection.launch INPUT_IMAGE:=/openni_camera/rgb/image_raw \
    model_file:=<MODEL_PATH> \
    label_file:=<LABEL_FILE_PATH>
```

![](./doc/recognition.gif)
