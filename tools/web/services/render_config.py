"""Camera requests are independent of robot commands and use world metres/radians."""
import math


def camera_defaults(spawn):
    return {'enabled':True, 'follow':True, 'azimuth':-2.35, 'elevation':.4,
            'distance':5., 'target':[spawn[0],spawn[1],.5]}


def validate_camera(value):
    if not isinstance(value,dict) or set(value) != {'enabled','follow','azimuth','elevation','distance','target'}:
        raise ValueError('Expected enabled, follow, azimuth, elevation, distance and target')
    if type(value['enabled']) is not bool or type(value['follow']) is not bool:
        raise ValueError('Camera enabled/follow must be boolean')
    for key,lo,hi in (('azimuth',-2*math.pi,2*math.pi),('elevation',.05,1.5),('distance',.5,250)):
        number=value[key]
        if type(number) not in (int,float) or not math.isfinite(number) or not lo<=number<=hi:
            raise ValueError('Invalid camera '+key)
    target=value['target']
    if not isinstance(target,list) or len(target)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>10000 for v in target):
        raise ValueError('Invalid camera target')
    return {**value,'target':list(target)}


def camera_pose(config,follow_position=None):
    target=list(follow_position if config['follow'] and follow_position is not None else config['target'])
    if config['follow'] and follow_position is not None: target[2]=max(.25,target[2])
    az,el,d=config['azimuth'],config['elevation'],config['distance']
    xyz=[target[0]+d*math.cos(el)*math.cos(az),target[1]+d*math.cos(el)*math.sin(az),target[2]+d*math.sin(el)]
    yaw=az+math.pi
    return xyz,[0.,el,yaw]
