import roslib
roslib.load_manifest('opencv2')
import cv

a = cv.LoadImage('/home/mkillpack/hrl_file_server/playpen_data_sets/2011-06-30_19-01-02/object000_try011_before_pr2.png', 0)
b = cv.LoadImage('/home/mkillpack/hrl_file_server/playpen_data_sets/2011-06-30_19-01-02/object000_try011_after_pr2.png', 0)
foreground = cv.CreateImage((640,480), 8, 1)
size = cv.GetSize(a)
IavgF = cv.CreateImage(size, cv.IPL_DEPTH_32F, 3)
IdiffF = cv.CreateImage(size, cv.IPL_DEPTH_32F, 3)
IprevF = cv.CreateImage(size, cv.IPL_DEPTH_32F, 3)
IhiF = cv.CreateImage(size, cv.IPL_DEPTH_32F, 3)
IlowF = cv.CreateImage(size, cv.IPL_DEPTH_32F, 3)
Ilow1 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
Ilow2 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
Ilow3 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
Ihi1 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
Ihi2 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
Ihi3 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
cv.Zero(IavgF)
cv.Zero(IdiffF)
cv.Zero(IprevF)
cv.Zero(IhiF)
cv.Zero(IlowF)

Icount = 0.00001

Iscratch = cv.CreateImage(size, cv.IPL_DEPTH_32F, 3)
Iscratch2 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 3)
Igray1 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
Igray2 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
Igray3 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
Imaskt = cv.CreateImage(size, cv.IPL_DEPTH_8U, 1)
cv.Zero(Iscratch)
cv.Zero(Iscratch2)
first = 1


def accumulateBackground(img):
    global first, Icount
    cv.CvtScale(img, Iscratch, 1, 0)
    if (not first):
        cv.Acc(Iscratch, IavgF)
        cv.AbsDiff(Iscratch, IprevF, Iscratch2)
        cv.Acc(Iscratch2, IdiffF)
        Icount += 1.0
    first = 0
    cv.Copy(Iscratch, IprevF)

def setHighThresh(thresh):
    cv.ConvertScale(IdiffF, Iscratch, thresh)
    cv.Add(Iscratch, IavgF, IhiF)
    cv.Split(IhiF, Ihi1, Ihi2, Ihi3, None)
    
def setLowThresh(thresh):
    cv.ConvertScale(IdiffF, Iscratch, thresh)
    cv.Sub(IavgF, Iscratch, IlowF)
    cv.Split(IlowF, Ilow1, Ilow2, Ilow3, None)


def createModelsfromStats():
    cv.ConvertScale(IavgF, IavgF, float(1.0/Icount))
    cv.ConvertScale(IdiffF, IdiffF, float(1.0/Icount))

    cv.AddS(IdiffF, cv.Scalar(1.0, 1.0, 1.0), IdiffF)
    setHighThresh(10.0)
    setLowThresh(10.0)

def backgroundDiff(img, Imask):
    cv.CvtScale(img, Iscratch, 1, 0)
    cv.Split(Iscratch, Igray1, Igray2, Igray3, None)
    cv.InRange(Igray1, Ilow1, Ihi1, Imask)

    cv.InRange(Igray2, Ilow2, Ihi2, Imaskt)
    cv.Or(Imask, Imaskt, Imask)

    cv.InRange(Igray3, Ilow3, Ihi3, Imaskt)
    cv.Or(Imask, Imaskt, Imask)

    cv.SubRS(Imask, 255, Imask)
    cv.SaveImage('/home/mkillpack/Desktop/mask.png', Imask)
    #cv.Erode(Imask, Imask)
    print "here is the sum of the non-zero pixels", cv.Sum(Imask)
    return Imask


if __name__ == '__main__':
    folder = '/home/mkillpack/hrl_file_server/playpen_data_sets/2011-06-30_19-01-02/'

    for j in [3]:  #[0, 3, 4, 5, 6]
        for i in xrange(200):
            try:
                file_name = folder+'object'+str(j).zfill(3)+'_try'+str(i).zfill(3)+'_after_pr2.png'
                img = cv.LoadImage(file_name, 1)
                print "reading ", file_name, '...'
            except:
                print file_name, " doesn't exist"
            if not img == None:
                accumulateBackground(img)

            #c = cv.LoadImage('/home/mkillpack/hrl_file_server/playpen_data_sets/2011-06-30_19-01-02/object000_try012_after_pr2.png', 1)
    createModelsfromStats()
    file_name = folder+'object006_try'+str(0).zfill(3)+'_before_pr2.png'
    img = cv.LoadImage(file_name, 3)    
    Imask = cv.CreateImage(size, cv.IPL_DEPTH_8U, 1)
    cv.Zero(Imask)
    cv.Zero(Imaskt)
    Imask = backgroundDiff(img, Imask)
