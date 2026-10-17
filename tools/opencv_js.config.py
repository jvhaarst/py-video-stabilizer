# Only what the stabilizer actually calls.
core = {
    '': ['absdiff', 'convertScaleAbs', 'copyMakeBorder', 'countNonZero',
         'minMaxLoc', 'normalize', 'split', 'merge', 'transpose'],
    'Algorithm': [],
}

imgproc = {
    '': ['cvtColor', 'resize', 'GaussianBlur', 'goodFeaturesToTrack',
         'warpAffine', 'threshold', 'Laplacian', 'Sobel'],
}

video = {
    '': ['calcOpticalFlowPyrLK', 'findTransformECC'],
}

# estimateAffinePartial2D is NOT in the upstream whitelist (opencv/opencv#20538).
# Adding it here is the whole point of a custom build.
calib3d = {
    '': ['estimateAffine2D', 'estimateAffinePartial2D', 'findHomography'],
}

white_list = makeWhiteList([core, imgproc, video, calib3d])
