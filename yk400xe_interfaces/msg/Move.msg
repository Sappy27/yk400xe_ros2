int8 move_type   # P=1, L=2 or C=3
int8 coords_type        # 0=Std Coordinates or 1=Pulse

float64[4] pose   # [x,y,z,thz] or [j1,j2,j3,j4]
float64[4] pose2  # for type C

int8 speed       # used as vel for type L and C

bool do_arch     # arch movement
float64[3] arch  # 3 element : [0]=height of the arch z direction, [1]&[2] width of the arch in x,y directions 

bool cont        # cont parameter (start next move as soon as cont distance to goal is reached)

