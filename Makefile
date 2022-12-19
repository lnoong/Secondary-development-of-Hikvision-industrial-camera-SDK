Demo: Grab_ImageCallback.cpp
	g++ -g -o Grab_ImageCallback Grab_ImageCallback.cpp -I../../../include -I/usr/local/include/opencv4  -Wl,-rpath=$(MVCAM_COMMON_RUNENV)/aarch64 -L/usr/local/lib -L/opt/MVS/lib/aarch64 -lMvCameraControl  -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_aruco -lopencv_imgcodecs
	

clean:
	rm Grab_ImageCallback -rf

