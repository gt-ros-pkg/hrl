#!/usr/bin/env python

from xml.dom import minidom

arm_joint_names = ['lr_shoulder_pan_joint',
                   'lr_shoulder_lift_joint',
                   'lr_upper_arm_roll_joint',
                   'lr_elbow_flex_joint',
                   'lr_forearm_roll_joint',
                   'lr_wrist_flex_joint',
                   'lr_wrist_roll_joint']

l_arm_joint_names = map(lambda s: s.replace('lr_', 'l_'), arm_joint_names )
r_arm_joint_names = map(lambda s: s.replace('lr_', 'r_'), arm_joint_names )

urdf_fname = '../config/pr2.urdf'
yaml_fname = '../config/r_arm_joint_limits.yaml'
joint_names = r_arm_joint_names

print( 'parsing %s ...'%urdf_fname )
xmldoc = minidom.parse( urdf_fname )
joint_nodes = xmldoc.getElementsByTagName('joint')
print( 'found %d joints'%len( joint_nodes ) )

print( 'writin to %s '%yaml_fname )
yaml_file = open( yaml_fname, 'w')

for node in joint_nodes:
    if not node.hasAttribute( 'name' ):
        continue
    joint_name = node.getAttribute( 'name' )
    if joint_name not in joint_names:
        continue
    print( 'found joint %s'%joint_name )

    limit_nodes = node.getElementsByTagName( 'limit' )
    if len( limit_nodes ) == 0:
        print( '\t no limit tag -> ignored' )
        continue

    if len( limit_nodes ) > 1:
        print( '\t more than one limit tag -> ignored' )
        continue
    
    effort = None
    velocity = None
    upper = None
    lower = None

    limit_node = limit_nodes[0]
    if limit_node.hasAttribute( 'effort' ):
        effort = limit_node.getAttribute( 'effort' )
    if limit_node.hasAttribute( 'velocity' ):
        velocity = limit_node.getAttribute( 'velocity' )
    if limit_node.hasAttribute( 'upper' ):
        upper = limit_node.getAttribute( 'upper' )
    if limit_node.hasAttribute( 'lower' ):
        lower = limit_node.getAttribute( 'lower' )

    safety_nodes = node.getElementsByTagName( 'safety_controller' )
    if len( safety_nodes ) == 0:
        print( '\t no safety_controller tag -> ignored' )
        continue

    if len( safety_nodes ) > 1:
        print( '\t more than one safety_controller tag -> ignored' )
        continue

    soft_lower_limit = None
    soft_upper_limit = None

    safety_node = safety_nodes[0]
    if safety_node.hasAttribute( 'soft_upper_limit' ):
        soft_upper_limit = safety_node.getAttribute( 'soft_upper_limit' )
    if safety_node.hasAttribute( 'soft_lower_limit' ):
        soft_lower_limit = safety_node.getAttribute( 'soft_lower_limit' )

    print( '%s:\n\teffort: %s\n\tvelocity: %s\n\tupper: %s\n\tlower: %s\n\tsoft_upper: %s\n\tsoft_lower: %s'%( joint_name, effort, velocity, upper, lower, soft_upper_limit, soft_lower_limit ) )

    yaml_file.write( '%s:\n'%joint_name )
    if not effort is None:
        yaml_file.write( '    effort: %s\n'%effort )
    if not velocity is None:
        yaml_file.write( '    velocity: %s\n'%velocity )
    if not upper is None:
        yaml_file.write( '    upper: %s\n'%upper )
    if not lower is None:
        yaml_file.write( '    lower: %s\n'%lower )
    if not soft_upper_limit is None:
        yaml_file.write( '    soft_upper_limit: %s\n'%soft_upper_limit )
    if not soft_lower_limit is None:
        yaml_file.write( '    soft_lower_limit: %s\n'%soft_lower_limit )

yaml_file.close()
    
        




