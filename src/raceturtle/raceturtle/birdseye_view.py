import cv2
import numpy as np

def main():
    print('Hi from raceturtle.')

    #calculate 'destination' image
    dst = transform(src_vertices, dst_vertices, src)
    
    #show 'source' and 'destination' images
    cv2.imshow("src", src)
    cv2.imshow("dst", dst)

    #close test windows using any key
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

#function to perform perspective transform using open cv
def transform(src_vertices, dst_vertices, src):
    #get transformation matrix
    M = cv2.getPerspectiveTransform(src_vertices, dst_vertices)
    #perform perspective transform -> save to dst 
    dst = cv2.warpPerspective(src, M, (640, 480))
    #return transformed image
    return dst

#load test image as 'src'
src = cv2.imread("src/images/test.jpg")

#define source and destination vertices for transform (to be adjusted based on camera view)
src_vertices = np.float32([
    [493, 443],
    [696, 440],
    [865, 646],
    [310, 646]
])

#define destination vertices for transform
dst_vertices = np.float32([
    [0, 0],
    [640, 0],
    [640, 480],
    [0, 480]
])
