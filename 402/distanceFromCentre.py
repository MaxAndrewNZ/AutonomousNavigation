
import pyrealsense2 as rs
import numpy as np
import cv2

path_to_bag = "../../../bags/room_static.bag"  # Location of input file.

# Decide if using a circle or rectangular averaging mask
isCircleMask = True
print("Using circle mask." if isCircleMask else "Using rectangle mask.")

# Radius of the averaging circle
radius = 100
# Dimensions of average rectangle
rWidth = 200
rHeight = 400

config = rs.config()
# This specifies that we are loading pre-recorded data 
# rather than using a live camera.
config.enable_device_from_file(path_to_bag)

pipeline = rs.pipeline()  # Create a pipeline
profile = pipeline.start(config)  # Start streaming
# Saving to a .bag file is done before alignment, so we need to do it when
# reading from .bag files too.
align = rs.align(rs.stream.color)

depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
print("Depth Scale is: {:.4f}m".format(depth_scale))

while True: # Loop over the images until we decide to quit.
    # Get the next frameset.
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    # Extract the frame images.
    depth_image = np.asanyarray(aligned_frames.get_depth_frame().get_data())
    color_image = np.asanyarray(aligned_frames.get_color_frame().get_data())

    # Create a new 8-bit colour version of the depth data for drawing on later.
    # This is scaled for visibility (65536 / 3500 ~= 18)
    depth_frame_scaled = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)

    # Do a simple "in range" threshold to find all objects between 2 and 5 units of distance away.
    # Note that each increment is equal to approximately 256*depths_scale. This is because the 
    # inRange function only accepts 8-bit integers, so we must scale it down.
    thresh = cv2.inRange((depth_image / 256).astype(np.uint8), 2, 5)

    # Dimensions of the frame
    height, width, depth = depth_frame_scaled.shape

    # Center of image
    cX = int(width / 2)
    cY = int(height / 2)

    # Create mask
    mask = np.zeros((height, width), np.uint8)
    if isCircleMask:
        cv2.circle(mask, (cX, cY), radius, 255, thickness=-1)
    else:
        rStartPoint = (int(cX - rWidth / 2), int(cY - rHeight / 2))
        rEndPoint = (int(cX + rWidth / 2), int(cY + rHeight / 2))
        cv2.rectangle(mask, rStartPoint, rEndPoint, 255, thickness=-1)
    
    masked_data = cv2.bitwise_and(depth_frame_scaled, depth_frame_scaled, mask=mask)

    # The distance values
    distanceToCentre = depth_frame_scaled[cX, cY][0] * depth_scale
    distanceAverage = cv2.mean(depth_frame_scaled, mask)[0] * depth_scale

    # Join the colour and the masked depth image
    jointImage = cv2.addWeighted(np.asarray(color_image / 256), 1.0, np.asarray(masked_data / 256), 0.025, 0.0)

    # Draw the center dot on the joint image.
    cv2.circle(jointImage, (cX, cY), 4, (0, 0, 255), -1)

    # Draw text for displaying distance
    cv2.putText(jointImage, '{:.2f}'.format(distanceToCentre.round(2)) + "m", (cX + 10, cY), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 0, 255), 1)

    # Draw the mask and text on the joint image.
    if isCircleMask:
        cv2.circle(jointImage, (cX, cY), radius, (0, 0, 255), 2)
        cv2.putText(jointImage, '{:.2f}'.format(round(distanceAverage, 2)) + "m mean", 
        (cX - radius + 10, cY + radius + 30), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 0, 255), 1)
    else:
        cv2.rectangle(jointImage, rStartPoint, rEndPoint, (0, 0, 255), 2)
        cv2.putText(jointImage, '{:.2f}'.format(round(distanceAverage, 2)) + "m mean", 
        (cX + int(rWidth / 2) + 30, cY), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 0, 255), 1)

    # cv2.imshow('depth', depth_frame_scaled)
    # cv2.imshow('color', color_image)
    # cv2.imshow("masked", masked_data)
    cv2.imshow("Joint", jointImage)
    # cv2.imshow("mask", mask)

    if cv2.waitKey(30) & 0xFF == ord('q'):
        break
