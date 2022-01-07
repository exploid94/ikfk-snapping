import pymel.core as pm
import maya.OpenMaya as om


def ik_to_fk(fk_list, ikm_list, ik_handle, ik_pole, ik_offset, ik_ball, blend_ctl, ik_rotation=True, key=True):
    """Snaps the ik handle control to the fk control and places the pole vector.
    Positions are calculated by getting the world space vectors of the fk controls
    and applying them to the ik controls.

    :param fk_list: A list of the fk controls in order. i.e ["shoulderFk_L0_001_ctl", "elbowFk_L0_001_ctl", "wristFk_L0_001_ctl"]
        Can be a list of strings (names) or selected pymel objects.
    :type fk_list: list

    :param ikm_list: A list of the ikm joints in order. i.e ["arm_L0_001_ikm", "arm_L0_002_ikm", "arm_L0_003_ikm"]
        Can be a list of strings (names) or selected pymel objects.
    :type ikm_list: list

    :param ik_handle: The control used to place the ik handle. i.e "handIk_L0_001_ctl"
        Can be a string (name) or a selected pymel object.
    :type ik_handle: str
    
    :param ik_pole: The control used as the pole vector. i.e "armPoleVectorIk_L0_001_ctl"
        Can be a string (name) or a selected pymel object.
    :type ik_pole: str
    
    :param blend_ctl: The control with the ikfk blend attr. i.e "armSettings_L0_001_ctl"
        Can be a string (name) or a selected pymel object.
    :type blend_ctl: str
    
    :param ik_rotation: Whether or not to orient the ik control. (True)
    :type ik_rotation: bool

    :param key: Whether or not to key the controls. (True)
    :type key: bool
    """
    
    # store blend attr
    blend_attr = pm.getAttr(blend_ctl+".blend")
    
    # reset roll attr
    pm.setAttr(ik_handle+".ikRoll", 0.0)
    
    # get fk transforms
    fk_001_translate = pm.xform(fk_list[0], ws=True, t=True, q=True)
    fk_001_pos = om.MVector(fk_001_translate[0], fk_001_translate[1], fk_001_translate[2])
    
    fk_002_translate = pm.xform(fk_list[1], ws=True, t=True, q=True)
    fk_002_pos = om.MVector(fk_002_translate[0], fk_002_translate[1], fk_002_translate[2])
    
    fk_003_translate = pm.xform(fk_list[2], ws=True, t=True, q=True)
    fk_003_pos = om.MVector(fk_003_translate[0], fk_003_translate[1], fk_003_translate[2])
   
    # move ik handle
    if ik_rotation:
        snap(ik_handle, ikm_list[2], blend_ctl+".blend", 1)
    else:
        pm.move(fk_003_pos.x, fk_003_pos.y, fk_003_pos.z, ik_handle)
    
    # find the pole position based on fk vector
    root_mid = fk_002_pos - fk_001_pos
    mid_goal = fk_003_pos - fk_002_pos
    root_goal = fk_003_pos - fk_001_pos
    upper_length = root_mid.length()
    lower_length = mid_goal.length()
    total_length = upper_length + lower_length

    root_goal_norm = root_goal.normal()
    root_mid_norm = root_mid / upper_length 

    mid_world = pm.xform(fk_list[1], ws=True, matrix=True, q=True)

    # move the ik pole
    pm.xform(ik_pole, ws=True, matrix=mid_world)
    #negate = -1.0 if "_R" in ik_handle.name() else 1.0
    negate = 1

    # compare the dot of the upper arm to the root->goal vector
    # If it's too close to 1, change the offset vector
    upper_goal_angle = root_mid_norm * root_goal_norm
    if upper_goal_angle > 0.998: ## this is arbitrary but works
        pole_y = om.MVector(0,negate,0) * mid_world
    else:
        goal_project = fk_001_pos + (root_goal_norm * upper_length * upper_goal_angle)
        pole_y = (fk_002_pos - goal_project).normal()

    pm.move(ik_pole, pole_y * total_length, r=True, ws=True)
    ik_pole.rotate.set(0, 0, 0)


    ######## anything beyond this point relates to the foot only ########
    if "leg" in str(blend_ctl):

        ball_ik = ik_ball
        ball_fk = fk_list[-1]

        # orient the ik to the fk
        ball_matrix = pm.xform(ball_fk, ws=True, matrix=True, q=True)
        pm.xform(ball_ik, ws=True, matrix=ball_matrix)


    # set the blend back to original state
    pm.setAttr(blend_ctl+".blend", blend_attr)

    # set keyframes
    if key:
        pm.setKeyframe([ik_handle, ik_pole])
        if ball_ik:
            pm.setKeyframe(ball_ik)
   

def fk_to_ik(fk_list, fk_offset_list, ikm_list, ik_ball, blend_ctl, key=True):
    """Snaps the fk controls to the ikm bones.

    :param fk_list: A list of the fk controls in order. i.e ["shoulderFk_L0_001_ctl", "elbowFk_L0_001_ctl", "wristFk_L0_001_ctl"]
        Can be a list of strings (names) or selected pymel objects.
    :type fk_list: list

    :param ikm_list: A list of the ikm joints in order. i.e ["arm_L0_001_ikm", "arm_L0_002_ikm", "arm_L0_003_ikm"]
        Can be a list of strings (names) or selected pymel objects.
    :type ikm_list: list
    
    :param blend_ctl: The control with the ikfk blend attr. i.e "armSettings_L0_001_ctl"
        Can be a string (name) or a selected pymel object.
    :type blend_ctl: str
    
    :param key: Whether or not to key the controls. (True)
    :type key: bool
    """

    blend_attr = pm.getAttr(blend_ctl+".blend")

    snap(fk_list[0], ikm_list[0], blend_ctl+".blend", 0)
    snap(fk_list[1], ikm_list[1], blend_ctl+".blend", 0)
    snap(fk_list[2], ikm_list[2], blend_ctl+".blend", 0)

    ######## anything beyond this point relates to the foot only ########
    if "leg" in str(blend_ctl):

        ball_ikm = ikm_list[-1]
        ball_fk = fk_list[-1]

        # orient the ik to the fk
        snap(ball_fk, ball_ikm, blend_ctl+".blend", 0)

    pm.setAttr(blend_ctl+".blend", blend_attr)

    # set keyframes
    if key:
        pm.setKeyframe(fk_list)
        if ball_fk:
            pm.setKeyframe(ball_fk)


def smart_ik_to_fk(ik_rotation=True, key=True):
    """Snaps the ik to the fk controls using message connections within the rig.

    :param ik_rotation: Whether or not to orient the ik control. (True)
    :type ik_rotation: bool

    :param key: Whether or not to key the controls. (True)
    :type key: bool
    """

    smart_list = get_rig_connections()

    fk_list = smart_list[0]
    fk_offset_list = smart_list[1]
    ik_handle = smart_list[2]
    ik_pole = smart_list[3]
    ik_offset = smart_list[4]
    blend_ctl = smart_list[5]
    ikm_list = smart_list[6]
    ik_ball = smart_list[7]
    
    ik_to_fk(fk_list, ikm_list, ik_handle, ik_pole, ik_offset, ik_ball, blend_ctl, ik_rotation=ik_rotation, key=key)


def smart_fk_to_ik(key=True):
    """Snaps the fk to the ik controls using message connections within the rig.

    :param key: Whether or not to key the controls. (True)
    :type key: bool
    """

    smart_list = get_rig_connections()

    fk_list = smart_list[0]
    fk_offset_list = smart_list[1]
    ik_handle = smart_list[2]
    ik_pole = smart_list[3]
    ik_offset = smart_list[4]
    blend_ctl = smart_list[5]
    ikm_list = smart_list[6]
    ik_ball = smart_list[7]
    
    fk_to_ik(fk_list, fk_offset_list, ikm_list, ik_ball, blend_ctl, key=key)


def get_rig_connections(obj=None):
    """Gets the message connections of the selected object if there are any.
    If there are no message connections, will try to find the approriate objects.

    :param obj: A specified object with the snapping attributes. If non, will get selected. (None)
    :type obj: str or obj
    """

    if not obj:
        obj = pm.ls(sl=True)[0]
    
    # make sure the current selected object has the ikfk blend attr
    # this control will have the message connections as well
    if pm.attributeQuery("blend", node=obj, exists=True):
        
        # find the fk controls message connections
        # fall back on finding controls if no message connection is in rig
        if pm.attributeQuery("fk_001", node=obj, exists=True):
            fk_001 = pm.listConnections(obj+".fk_001")[0]
        else:
            fk_001 = None

        if pm.attributeQuery("fk_002", node=obj, exists=True):
            fk_002 = pm.listConnections(obj+".fk_002")[0]
        else:
            fk_002 = None

        if pm.attributeQuery("fk_003", node=obj, exists=True):
            fk_003 = pm.listConnections(obj+".fk_003")[0]
        else:
            fk_003 = None
        
        if pm.attributeQuery("fk_005", node=obj, exists=True):
            fk_005 = pm.listConnections(obj+".fk_005")[0]
        else:
            fk_005 = None

        if pm.attributeQuery("fk_001_offset", node=obj, exists=True):
            fk_001_offset = pm.listConnections(obj+".fk_001_offset")[0]
        else:
            fk_001_offset = None

        if pm.attributeQuery("fk_002_offset", node=obj, exists=True):
            fk_002_offset = pm.listConnections(obj+".fk_002_offset")[0]
        else:
            fk_002_offset = None

        if pm.attributeQuery("fk_003_offset", node=obj, exists=True):
            fk_003_offset = pm.listConnections(obj+".fk_003_offset")[0]
        else:
            fk_003_offset = None
        
        # find the ik controls message connections
        # fall back on finding controls if no message connection is in rig
        if pm.attributeQuery("ik_handle", node=obj, exists=True):
            ik_handle = pm.listConnections(obj+".ik_handle")[0]
        else:
            ik_handle = None

        if pm.attributeQuery("ik_pole", node=obj, exists=True):
            ik_pole = pm.listConnections(obj+".ik_pole")[0]
        else:
            ik_pole = None
        
        if pm.attributeQuery("ik_offset", node=obj, exists=True):
            ik_offset = pm.listConnections(obj+".ik_offset")[0]
        else:
            ik_offset = None
        
        if pm.attributeQuery("ik_ball", node=obj, exists=True):
            ik_ball = pm.listConnections(obj+".ik_ball")[0]
        else:
            ik_ball = None
        
        # find the ikm bones message connections
        # fall back on finding joints if no message connection is in rig
        if pm.attributeQuery("ikm_001", node=obj, exists=True):
            ikm_001 = pm.listConnections(obj+".ikm_001")[0]
        else:
            ikm_001 = None

        if pm.attributeQuery("ikm_002", node=obj, exists=True):
            ikm_002 = pm.listConnections(obj+".ikm_002")[0]
        else:
            ikm_002 = None

        if pm.attributeQuery("ikm_003", node=obj, exists=True):
            ikm_003 = pm.listConnections(obj+".ikm_003")[0]
        else:
            ikm_003 = None
        
        if pm.attributeQuery("ikm_006", node=obj, exists=True):
            ikm_006 = pm.listConnections(obj+".ikm_006")[0]
        else:
            ikm_006 = None

    return [fk_001, fk_002, fk_003, fk_005], [fk_001_offset, fk_002_offset, fk_003_offset], ik_handle, ik_pole, ik_offset, obj, [ikm_001, ikm_002, ikm_003, ikm_006], ik_ball


def snap(source, target, attr, value):
    """Snaps the source to the target by duplicating a dummy of the source and moving it with the ik blend

    :param source: The control to move to the target. Can be a string (name) or a selected pymel object.
    :type source: str
    
    :param target: The target to move to the source to. Can be a string (name) or a selected pymel object.
    :type target: str
    
    :param attr: The node and attribute of the ikfk blend.
    :type attr: str

    :param value: The starting ikfk blend. If an fk control is the source, then this value should be 0. 
        If the source is an ik control, then this value should be 1. 
    :type value: float
    """

    # set the attr value first
    pm.setAttr(attr, value)

    # duplicate source and and store parent
    dup_source = pm.duplicate(source, po=True)[0]
    source_parent = pm.listRelatives(source, parent=True)[0]

    # parent dup_source to target
    pm.parent(dup_source, target)

    # invert attr
    pm.setAttr(attr, 1-value)

    # transfer transforms from dup_source to source
    transforms = pm.xform(dup_source, ws=True, matrix=True, q=True)
    pm.xform(source, ws=True, matrix=transforms)

    # delete dup
    pm.delete(dup_source)


def snap_pole_vector_to_elbow(key=True):
    """Snaps the pole vector to the elbow joint.

    :param key: Whether or not to key the pole vector in place. (True)
    :type key: bool
    """

    smart_list = get_rig_connections()

    # get the ik objects
    ik_handle = smart_list[2]
    ik_pole = smart_list[3]
    ikm_list = smart_list[6]
    ikm_002 = ikm_list[1]

    # turn off ikPin
    if ik_handle:
        pm.setAttr(ik_handle.ikPin, 0.0)

    # get the elbow position in world space
    if ikm_002:
        ikm_002_trans = pm.xform(ikm_002, ws=True, translation=True, q=True)
    
    # set the pole vector to the elbow position
    if ik_pole and ikm_002:
        pm.xform(ik_pole, ws=True, translation=ikm_002_trans)
    
    # turn on ikPin
    if ik_handle:
        pm.setAttr(ik_handle.ikPin, 1.0)
    
    if key:
        pm.setKeyframe(ik_pole)


def add_connections(namespace=None): 
    """Creates message connections on controls so we can easily know which objects in the scene are ik and fk objects.
    This is used for the ikfk-snapping tool and will only be setup once per rig, just need to have names be a standard the first time.
    After we set the names in these dictionaries, all rigs should be consistent and should work with running this once.

    :param namespace: Namespace of objects to add the connections on. (None)
    :type namespace: str
    """
    L_arm_dict = {
                    "fk_001":"shoulderFk_L0_001_ctl",
                    "fk_002":"elbowFk_L0_001_ctl",
                    "fk_003":"wristFk_L0_001_ctl",
                    "fk_001_offset":"armFkOffset_L0_001_loc",
                    "fk_002_offset":"armFkOffset_L0_002_loc",
                    "fk_003_offset":"armFkOffset_L0_002_loc",
                    "ik_base":"armIk_L0_001_ctl",
                    "ik_handle":"handIk_L0_001_ctl",
                    "ik_pole":"armPoleVectorIk_L0_001_ctl",
                    "ik_offset":"ikHandOffset_arm_L0_loc",
                    "ikm_001":"arm_L0_001_ikm",
                    "ikm_002":"arm_L0_002_ikm",
                    "ikm_003":"arm_L0_003_ikm",
                }

    R_arm_dict = {
                    "fk_001":"shoulderFk_R0_001_ctl",
                    "fk_002":"elbowFk_R0_001_ctl",
                    "fk_003":"wristFk_R0_001_ctl",
                    "fk_001_offset":"armFkOffset_R0_001_loc",
                    "fk_002_offset":"armFkOffset_R0_002_loc",
                    "fk_003_offset":"armFkOffset_R0_002_loc",
                    "ik_base":"armIk_R0_001_ctl",
                    "ik_handle":"handIk_R0_001_ctl",
                    "ik_pole":"armPoleVectorIk_R0_001_ctl",
                    "ik_offset":"ikHandOffset_arm_R0_loc",
                    "ikm_001":"arm_R0_001_ikm",
                    "ikm_002":"arm_R0_002_ikm",
                    "ikm_003":"arm_R0_003_ikm",
                }

    L_leg_dict = {
                    "fk_001":"hipFk_L0_001_ctl",
                    "fk_002":"kneeFk_L0_001_ctl",
                    "fk_003":"ankleFk_L0_001_ctl",
                    "fk_005":"legFk_L0_005_ctl",
                    "fk_001_offset":"legFkOffset_L0_001_loc",
                    "fk_002_offset":"legFkOffset_L0_002_loc",
                    "fk_003_offset":"legFkOffset_L0_003_loc",
                    "ik_base":"legIk_L0_001_ctl",
                    "ik_handle":"footIk_L0_001_ctl",
                    "ik_pole":"legPoleVectorIk_L0_001_ctl",
                    "ik_offset":"ikFootOffset_leg_L0_loc",
                    "ik_ball":"legBallIk_L0_001_ctl",
                    "ikm_001":"leg_L0_001_ikm",
                    "ikm_002":"leg_L0_002_ikm",
                    "ikm_003":"leg_L0_003_ikm",
                    "ikm_006":"leg_L0_006_ikm",
                }

    R_leg_dict = {
                    "fk_001":"hipFk_R0_001_ctl",
                    "fk_002":"kneeFk_R0_001_ctl",
                    "fk_003":"ankleFk_R0_001_ctl",
                    "fk_005":"legFk_R0_005_ctl",
                    "fk_001_offset":"legFkOffset_R0_001_loc",
                    "fk_002_offset":"legFkOffset_R0_002_loc",
                    "fk_003_offset":"legFkOffset_R0_003_loc",
                    "ik_base":"legIk_R0_001_ctl",
                    "ik_handle":"footIk_R0_001_ctl",
                    "ik_pole":"legPoleVectorIk_R0_001_ctl",
                    "ik_offset":"ikFootOffset_leg_R0_loc",
                    "ik_ball":"legBallIk_R0_001_ctl",
                    "ikm_001":"leg_R0_001_ikm",
                    "ikm_002":"leg_R0_002_ikm",
                    "ikm_003":"leg_R0_003_ikm",
                    "ikm_006":"leg_R0_006_ikm",
                }


    controls = {"armSettings_L0_001_ctl":L_arm_dict, 
                "armSettings_R0_001_ctl":R_arm_dict, 
                "legSettings_L0_001_ctl":L_leg_dict, 
                "legSettings_R0_001_ctl":R_leg_dict}


    for obj in controls:      
        for attr in controls[obj]:
            try:
                rig_obj = controls[obj][attr]
                
                if namespace:
                    obj_name = namespace+":"+obj
                    rig_obj_name = namespace+":"+rig_obj
                else:
                    obj_name = obj
                    rig_obj_name = rig_obj

                # create attr if not already exists
                if not cmds.attributeQuery(attr, node=obj_name, exists=True):
                    cmds.addAttr(obj_name, ln=attr, at="message")
                
                # connect message to attr based on dictionary
                cmds.connectAttr(rig_obj_name+".message", obj_name+"."+attr, force=True)

            except:
                raise
                