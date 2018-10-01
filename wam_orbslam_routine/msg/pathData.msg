# This message is used to specify a RotatedRect within an image.
# This RotatedRect is represented by the position (x,y) of its COG, its area (a) and its orientation (theta in degrees) around the optical axis z


float32 x  # x position of the COG of the box
float32 y  # y position of the COG of the box
float32 a    # area of the box
float32 theta	 # orientation in degrees of the box around the camera optical axis 
float32 targetx #x position target
float32 targety #y position target
int8    savingFlag #saving flag target position
int16[] u_path
int16[] v_path

