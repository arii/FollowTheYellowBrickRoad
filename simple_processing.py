from PIL import Image
im = Image.open("testy.mjpeg")
pix = im.load()

X,Y = im.size

avg_x = 0
hits = 0
for x in range(X):
    for y in range(Y):
        if sum(pix[x,y])> 240*3 :
            pix[x,y] = (0,0,0)
            avg_x += x
            hits += 1

if hits > 0:
    avg_x =avg_x/hits
    if avg_x < X/2:
        direction = "LEFT"
    else:
        direction = "RIGHT"

    print "\n\n\n\n **INSTRUCTIONS \n\n\naverage x is %s.  Go %s" % (avg_x, direction)

else:
    print "location not found!"
