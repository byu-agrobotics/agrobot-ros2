import rclpy
from rclpy.node import Node
from agrobot_interfaces.srv import IdentifyEgg
import cv2
import numpy as np
import threading
from queue import Queue
import os

class EggID(Node):
    '''
    :author: Nelson Durrant, Ashton Palacios
    :date: November 2024

    Service that identifies eggs.

    Services:
        - egg/identify (agrobot_interfaces/srv/IdentifyEgg)
    '''

    def __init__(self):
        super().__init__('egg_id')

        self.egg_id_service = self.create_service(IdentifyEgg, 'egg/identify', self.egg_id_callback)
        self.egg_camera = cv2.VideoCapture(8)

        # Tune the exposure and white balance values here to get the colors to be what they need to be
        os.system("v4l2-ctl -d 8 -c auto_exposure=1,exposure_dynamic_framerate=0,white_balance_automatic=0,white_balance_temperature=3300,exposure_time_absolute=500")
        self.color_ranges = {
            "good": ((70,23,11),(100,209,237)),
            "bad": ((54,40,26),(77,134,221))
        }

        # Thread needed to continously pull frames from the camera or we get old frames
        # TODO: thread needs a better way of dying
        self.q = Queue()
        self.imageCount = 0
        self.thread = threading.Thread(target=self._reader)
        self.thread.daemon = False
        self.thread.start()

    def _reader(self):
        # Camera thread that continously pulls frame and puts them on a Queue
        while True:
            ret, frame = self.egg_camera.read()
            if not ret:
                break
            if not self.q.empty():
                try:
                    self.q.get_nowait() # discard previous (unprocessed) frame
                except queue.Empty:
                    pass
            self.q.put(frame)

    def read(self):
        # Get a frame from the camera Queue, should be the most recent frame
        return self.q.get()
        

    def egg_id_callback(self, request, response):
        '''
        Callback function for the egg identification service.
        
        :param request: Request message
        :type request: agrobot_interfaces.srv.IdentifyEgg.Request
        :param response: Response message
        :type response: agrobot_interfaces.srv.IdentifyEgg.Response
        '''

        self.get_logger().info('Received request to identify an egg!')
        image = self.read()

        # Helpful debugging code
        # self.get_logger().info("CV_CAP_PROP_FRAME_WIDTH: '{}'".format(self.egg_camera.get(cv2.CAP_PROP_FRAME_WIDTH))) 
        # self.get_logger().info("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.egg_camera.get(cv2.CAP_PROP_FRAME_HEIGHT))) 
        # self.get_logger().info("CAP_PROP_FPS : '{}'".format(self.egg_camera.get(cv2.CAP_PROP_FPS))) 
        # self.get_logger().info("CAP_PROP_POS_MSEC : '{}'".format(self.egg_camera.get(cv2.CAP_PROP_POS_MSEC))) 
        # self.get_logger().info("CAP_PROP_FRAME_COUNT  : '{}'".format(self.egg_camera.get(cv2.CAP_PROP_FRAME_COUNT))) 
        # self.get_logger().info("CAP_PROP_BRIGHTNESS : '{}'".format(self.egg_camera.get(cv2.CAP_PROP_BRIGHTNESS))) 
        # self.get_logger().info("CAP_PROP_CONTRAST : '{}'".format(self.egg_camera.get(cv2.CAP_PROP_CONTRAST))) 
        # self.get_logger().info("CAP_PROP_SATURATION : '{}'".format(self.egg_camera.get(cv2.CAP_PROP_SATURATION))) 
        # self.get_logger().info("CAP_PROP_HUE : '{}'".format(self.egg_camera.get(cv2.CAP_PROP_HUE))) 
        # self.get_logger().info("CAP_PROP_GAIN  : '{}'".format(self.egg_camera.get(cv2.CAP_PROP_GAIN))) 
        # self.get_logger().info("CAP_PROP_CONVERT_RGB : '{}'".format(self.egg_camera.get(cv2.CAP_PROP_CONVERT_RGB)))
        # self.get_logger().info("CAP_PROP_AUTOFOCUS : '{}'".format(self.egg_camera.get(cv2.CAP_PROP_AUTOFOCUS)))
        # self.get_logger().info("CAP_PROP_AUTO_WB : '{}'".format(self.egg_camera.get(cv2.CAP_PROP_AUTO_WB)))
        # self.get_logger().info("CAP_PROP_EXPOSURE  : '{}'".format(self.egg_camera.get(cv2.CAP_PROP_AUTO_EXPOSURE )))
        # self.get_logger().info("CAP_PROP_EXPOSURE  : '{}'".format(self.egg_camera.get(cv2.CAP_PROP_EXPOSURE )))
        # self.get_logger().info("CAP_PROP_WB_TEMPERATURE : '{}'".format(self.egg_camera.get(cv2.CAP_PROP_WB_TEMPERATURE)))

        self.imageCount += 1

        # Convert the image into the desired space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        masks = {egg_type: cv2.inRange(hsv_image, lower, upper) for egg_type, (lower, upper) in self.color_ranges.items()}
        characterics = {"good": 0, "bad": 0}

        for egg_type, mask in masks.items():

            # Find the contours of the eggs
            kernel = np.ones((3, 3), np.uint8) # Define a 3x3 kernel; you can adjust the size as needed
            eroded_mask = cv2.erode(mask, kernel, iterations=5) # Apply erosion
            masked_image = cv2.bitwise_and(image, image, mask=eroded_mask)
            contours, _ = cv2.findContours(eroded_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self.get_logger().info(f"\tegg_type: {egg_type}")

            totalArea = 0
            for contour in contours:
                # Calculate the area of the contour (size of the egg)
                area = cv2.contourArea(contour)
                totalArea += area
                
                # Draw contours and label the egg on the image
                # Nice for debugging if you're saving the image
                cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
                cv2.putText(image, f"{egg_type} - {area}", tuple(contour[0][0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            self.get_logger().info(f"\t\tarea total: {totalArea}")
            characterics[egg_type] = totalArea

        # Save the image for debugging
        # Compare egg types
        egg_type = 3
        if characterics["bad"] > characterics["good"]:
            self.get_logger().info(f"\tWe have a bad egg")
            cv2.putText(image, f"BAD EGG", (image.shape[1]//2-75,image.shape[0]//2), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2)

        else: # good egg, find if medium or large
            if characterics["good"] > 113_000: # may have to tune this value depending on the system
                self.get_logger().info("\tWe have Large Good egg")
                cv2.putText(image, f"Large Good EGG", (image.shape[1]//2-300,image.shape[0]//2), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2)

                egg_type = 2
            else:
                self.get_logger().info("\tWe have a medium Good egg")
                cv2.putText(image, f"Medium Good EGG", (image.shape[1]//2-300,image.shape[0]//2), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2)

                egg_type = 1

        cv2.imwrite(f"/tmp/test{self.imageCount}.jpeg",image)
        # Return the egg type for the FSM
        # egg_type = 2
        response.egg_type = egg_type # 1: small, 2: large, 3: bad
        return response

def main(args=None):
    rclpy.init(args=args)

    egg_id_node = EggID()
    rclpy.spin(egg_id_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    egg_id_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
