import time
import math
import numpy as np



orgllh = [45.030893, 7.618976, 254]
theta_deg = 118.93                  # ISMB
theta = theta_deg/180*math.pi
PI = 3.1415926535898                # WGS-84 pi
SPEED_OF_LIGHT = 299792458.0        # WGS-84 vacuum speed of light in seconds
EARTH_SEMIMAJOR_AXIS = 6378137.0000        # WGS-84 semi-major axis of the reference ellipsoid in meters
EARTH_SEMIMINOR_AXIS = 6356752.3142        # WGS-84 semi-minor axis of the reference ellipsoid in meters
INVERSE_FLATTENING = 298.257223563        # WGS-84 inverse flattening of the reference ellipsoid
EARTH_ROTATION_RATE = 7.2921151467E-5        # WGS-84 earth's rotation rate
EARTH_GRAVITATION_PARAM = 3.986005E+14        # WGS 84 earth's universal gravitation parameter

#FUNCTIONS

# Local to ENU - OK
def xyz2enu(xyz,theta):
    matrixTran = np.array([[math.cos(theta), math.sin(theta)],[-math.sin(theta), math.cos(theta)]])
    x=xyz[0]
    y=xyz[1]
    z=xyz[2]
    xy=np.array([x,y])
    en=np.array([matrixTran.dot(xy)])
    enu=np.concatenate((en,z),axis=None,dtype=np.double)
    return enu
    
# DEG to RAD - OK
def deg2rad(deg):
    return (math.pi/180)*deg

# RAD to DEG - OK
def rad2deg(rad):
    return (rad*180)/math.pi

# LLH to XYZ - OK    
def llh2xyz(llh):
    a = 6378137.0000
    b = 6356752.3142
    phi = deg2rad(llh[0]) #was llh[:,1]
    lam = deg2rad(llh[1]) #was llh[:,2]

    sinphi = math.sin(phi)
    cosphi = math.cos(phi)
    coslam = math.cos(lam)
    sinlam = math.sin(lam)
    tan2phi = math.pow(math.tan(phi),2)

    c=b/a
    p=1-(math.pow(c,2))
    e = math.sqrt(p)
    tmp = 1 - (e*e)
    tmpden = math.sqrt( 1 + tmp*tan2phi)
    tmp2 = math.sqrt(1 - e*e*sinphi*sinphi)

    xyz=np.array([0,0,0])
    xyz[0] = (a*coslam)/tmpden + llh[2]*coslam*cosphi
    xyz[1] = (a*sinlam)/tmpden + llh[2]*sinlam*cosphi
    xyz[2] = (a*tmp*sinphi)/tmp2 + llh[2]*sinphi
    return xyz
    
# ENU to LLH

def enu2llh(xyz, orgllh):
    orgxyz = llh2xyz(orgllh)
    
    phi = deg2rad(orgllh[0])
    lam = deg2rad(orgllh[1])

    sinphi = math.sin(phi)
    cosphi = math.cos(phi)
    sinlam = math.sin(lam)
    coslam = math.cos(lam)
    eye3 = np.array([[1,0,0],[0,1,0],[0,0,1]],dtype=np.double)
    R1 = np.array([[-sinlam,-sinphi*coslam,cosphi*coslam],[coslam,-sinphi*sinlam,cosphi*sinlam],[0,cosphi,sinphi]])
    R1=np.linalg.inv(R1)
    R=R1.dot(eye3)
    temp=enu.dot(R)
    xyz=np.add(temp,orgxyz)
    a = 6378137.0
    b = 6356752.3142
    c=b/a
    p=1-(math.pow(c,2))
    e = math.sqrt(p)
    b2 = b*b
    e2 = e*e
    ep = e*(a/b)
    r2 = math.pow(xyz[0],2)+math.pow(xyz[1],2)
    r = math.sqrt(r2)
    z2 = math.pow(xyz[2],2)
    F = 54*b2*z2
    G = r2 + (1-e2)*z2 - e2*((a*a) - (b*b))
    c = (e2*e2*F*r2)/(G*G*G)
    d=c*c + 2*c
    s1 = 1 + c + math.sqrt(d)
    s=math.pow(s1, (1/3))
    k=math.pow((s+(1/s)+1),2)
    P = F / (3*k* G*G)
    Q = math.sqrt(1+2*e2*e2*P)
    #ok
    a1=-(P*e2*r)/(1+Q)
    a2=a*(a/2)*(1+(1/Q))
    a3=(P*(1-e2)*z2)/(Q*(1+Q))
    a4=P*(r2/2)
    ro = a1+math.sqrt(a2- a3 - a4)
    tmp = math.pow((r - e2*ro),2)
    V = math.sqrt( tmp + (1-e2)*z2 )
    b1=math.atan2(xyz[2] + ep*ep*(b2*xyz[2])/(a*V), r)
    llh=np.array([0,0,0],dtype=np.double)
    llh[0] = rad2deg(b1)
    llh[1] = rad2deg(math.atan2(xyz[1], xyz[0]))
    llh[2] = math.sqrt( tmp + z2 )*( 1 - b2/(a*V))
    return llh
    
    
#MAIN
x_pos=0
y_pos=-20
z_pos=1
matrixTran = np.array([[math.cos(theta), math.sin(theta)],[-math.sin(theta), math.cos(theta)]])
xy=np.array([x_pos,y_pos],dtype=np.float)
en=np.array([matrixTran.dot(xy)],dtype=np.float)
enu=np.concatenate((en,z_pos),axis=None)    
fin=enu2llh(enu, orgllh)
print("%.10f" % fin[0],"%.10f" % fin[1],"%.10f" % fin[2])







