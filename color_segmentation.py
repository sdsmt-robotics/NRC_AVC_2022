import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
from mpl_toolkits import mplot3d

img = cv.imread('bucket_1.jpg')
img_full = img.copy()
img = cv.resize(img,None,fx = 0.1,fy = 0.1,interpolation = cv.INTER_CUBIC)

img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
plt.imshow(img_rgb)
plt.show()

red_plane = img_hsv[:,:,0]
red_vector = red_plane.flatten()
green_plane = img_hsv[:,:,1]
green_vector = green_plane.flatten()
blue_plane = img_hsv[:,:,2]
blue_vector = blue_plane.flatten()

# blue bucket pixels
print("+==========Blue Bucket==========+")
bbpt1x = int(input('Pt 1 X: ')) #250
bbpt1y = int(input('Pt 1 Y: ')) #140

bbpt2x = int(input('Pt 2 X: ')) #350
bbpt2y = int(input('Pt 2 Y: ')) #250

bb_im = img_hsv[bbpt1y:bbpt2y, bbpt1x:bbpt2x,:]
rgb_rec = cv.rectangle(img_rgb, (bbpt1x, bbpt1y), (bbpt2x, bbpt2y), (0, 0, 255), 3)
bbrp = bb_im[:,:,0]
bbrpv = bbrp.flatten()
bbgp = bb_im[:,:,1]
bbgpv = bbgp.flatten()
bbbp = bb_im[:,:,2]
bbbpv = bbbp.flatten()

# red bucket pixels
print("+==========Red Bucket==========+")
rbpt1x = int(input('Pt 1 X: ')) #108
rbpt1y = int(input('Pt 1 Y: ')) #79

rbpt2x = int(input('Pt 2 X: ')) #128
rbpt2y = int(input('Pt 2 Y: ')) #95

rb_im = img_hsv[rbpt1y:rbpt2y, rbpt1x:rbpt2x,:]
rgb_rec = cv.rectangle(rgb_rec, (rbpt1x, rbpt1y), (rbpt2x, rbpt2y), (255, 0, 0), 3)
rbrp = rb_im[:,:,0]
rbrpv = rbrp.flatten()
rbgp = rb_im[:,:,1]
rbgpv = rbgp.flatten()
rbbp = rb_im[:,:,2]
rbbpv = rbbp.flatten()

# yellow bucket pixels
print("+==========Yellow Bucket==========+")
ybpt1x = int(input('Pt 1 X: ')) #192
ybpt1y = int(input('Pt 1 Y: ')) #79

ybpt2x = int(input('Pt 2 X: ')) #220
ybpt2y = int(input('Pt 2 Y: ')) #105

yb_im = img_hsv[ybpt1y:ybpt2y, ybpt1x:ybpt2x,:]
rgb_rec = cv.rectangle(rgb_rec, (ybpt1x, ybpt1y), (ybpt2x, ybpt2y), (255, 255, 0), 3)
ybrp = yb_im[:,:,0]
ybrpv = ybrp.flatten()
ybgp = yb_im[:,:,1]
ybgpv = ybgp.flatten()
ybbp = yb_im[:,:,2]
ybbpv = ybbp.flatten()

plt.imshow(rgb_rec)

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(red_vector,green_vector,blue_vector,'k.', alpha = 0.005)
ax.plot3D(bbrpv,bbgpv,bbbpv,'b.',alpha = 0.1)
ax.plot3D(rbrpv,rbgpv,rbbpv,'r.',alpha = 0.5)
ax.plot3D(ybrpv,ybgpv,ybbpv,'y.',alpha = 0.1)
ax.set_xlabel('Hue')
ax.set_ylabel('Sat')
ax.set_zlabel('Value')
plt.show()

print("+==========Test Color Ranges==========+")
while (True):
	cont = input('Y/N: ')
	if (cont == 'N') or (cont == 'n'):
		break
	print("+==========Lower Parameters==========+")

	hl = int(input('H: ')) #111
	sl = int(input('S: ')) #102
	vl = int(input('V: ')) #86

	print("+==========Upper Parameters==========+")

	hu = int(input('H: ')) #120
	su = int(input('S: ')) #202
	vu = int(input('V: ')) #116

	fig = plt.figure()
	lower_vals = np.array([111,102,86])
	upper_vals = np.array([120,202,116])
	# Threshold the image to get only blue bucket
	mask = cv.inRange(img_hsv, lower_vals, upper_vals)
	# Bitwise-AND mask and original image
	res = cv.bitwise_and(img_hsv,img_hsv, mask = mask)
	plt.imshow(res)

	plt.show()

