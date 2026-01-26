import cv2 as cv

img = cv.imread("dog.jpg")

# Downsample by factor 2
img_down = cv.resize(img, dsize=None, fx=0.5, fy=0.5, interpolation=cv.INTER_AREA)

print(f"original image size = {img.shape}, downsampled image = {img_down.shape}")
cv.imshow("Original image", img)
cv.imshow("Downsampled image", img_down)

cv.waitKey(0)
cv.destroyAllWindows()
