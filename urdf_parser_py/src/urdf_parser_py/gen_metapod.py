import roslib; roslib.load_manifest('urdf_parser_py')
import rospy

import sys

from urdf_parser_py.urdf import URDF


class GenerateMetapodFromURDF:
    def __init__(self, anURDF):
        self.urdf = anURDF
        self.nb_dof = 0

    def open_header(self,f,suffix):
        s='#ifndef ' + suffix + '_HH\n'
        f.write(s)
        s='#  define ' + suffix + '_HH\n'
        f.write(s)

    def open_metapod_ns(self,f):
        s='namespace metapod' + '\n' + '{\n'
        f.write(s)
        
    def close_metapod_ns(self,f):
        s='} // end of namespace metapod\n'
        f.write(s)

    def open_robot_ns(self,f,robot_name):
        s='  namespace ' + self.urdf.name + '\n' + '  {\n'
        f.write(s)
        
    def close_robot_ns(self,f):
        s='  } // end of namespace '+ self.urdf.name + '\n'
        f.write(s)
        
    def close_header(self,f,suffix):
        s='#endif /* ' + suffix + '_HH */'
        f.write(s)

    # Bodies related method
    def generate_body(self,f,body,joint,body_parent=None):
        s='    CREATE_BODY(' + ' ' + body
        if body_parent is None:
            s= s + ', 0, NP, ' 
            s= s + joint.name + ');\n'
        else:
            s= s + ', 1,' + body_parent 
            s= s + ', ' + joint + ');\n'
        f.write(s)

    def generate_bodies(self):
        lfilename = self.urdf.name + '_body.hh'
        f = open(lfilename,'w')
        header_suffix = self.urdf.name + '_BODY'   
        self.open_header(f,header_suffix)

        s='#include "metapod/tools/bodymacros.hh"\n'
        f.write(s)

        self.open_metapod_ns(f)
        self.open_robot_ns(f,self.urdf.name)        

        for k, v in self.urdf.links.iteritems():
            if k not in self.urdf.parent_map:
              self.generate_body(f, k, v)
            else:
              (joint,parent) = self.urdf.parent_map[k]
              if self.urdf.joints[joint].joint_type != 'fixed':
                  self.generate_body(f, k, joint, parent)

        self.close_robot_ns(f)
        self.close_metapod_ns(f)

        self.close_header(f,header_suffix)
        f.close()

    # Bodies related method
    def generate_joint(self,f,joint):
        towrite=False
        s = '      ';
        if joint.joint_type == 'floating':
          s=s+'JOINT_FREE_FLYER(' +joint.name + ');'
          towrite=True
        if joint.joint_type == 'revolute':
          laxis = joint.axis.split(' ')
          s=s+'JOINT_REVOLUTE_AXIS_ANY(' + joint.name + ','\
              + laxis[0] + ',' + laxis[1] + ','\
              + laxis[2] + ');'
          towrite=True
          self.nb_dof = self.nb_dof + 1

        s=s+'\n'
        if towrite:
            f.write(s)      

    def generate_joints(self):
        lfilename = self.urdf.name + '_joint.hh'
        f = open(lfilename,'w')
        header_suffix = self.urdf.name + '_JOINT'   
        self.open_header(f,header_suffix)

        s='#include "metapod/tools/jointmacros.hh"\n'
        f.write(s)

        self.open_metapod_ns(f)
        self.open_robot_ns(f,self.urdf.name)        

        for k, v in self.urdf.joints.iteritems():
            self.generate_joint(f,v)

        self.close_robot_ns(f)
        self.close_metapod_ns(f)

        self.close_header(f,header_suffix)
        f.close()

    def generate_template_robot(self):
        lfilename = self.urdf.name + '.hh'
        f = open(lfilename,'w')
        header_suffix = self.urdf.name    
        self.open_header(f,header_suffix)
        s='#include "metapod/tools/common.hh"\n'
        f.write(s)
        s='#include "metapod/algos/rnea.hh"\n'
        f.write(s)
        s='#include "metapod/algos/crba.hh"\n\n'
        f.write(s)
        s ='#include "robot.hh"\n\n'
        
        s='extern template struct metapod::crba<metapod::'+ self.urdf.name + ',true>;\n'
        f.write(s)
        s='extern template struct metapod::rnea<metapod::'+ self.urdf.name + ',true>;\n'
        f.write(s)      
        s='extern template struct metapod::crba<metapod::'+ self.urdf.name + ',false>;\n'
        f.write(s)      
        s='extern template struct metapod::rnea<metapod::'+ self.urdf.name + ',false>;\n\n'
        f.write(s)

        self.close_header(f,header_suffix)
        f.close()

    def generate_init_joint(self,f,joint):
        s_width='    ';
        s=s_width+ '// Init'+joint.name
        f.write(s)
        
        # Initialize joint
        if joint.joint_type == 'floating':
          s=s_width+'INITIALIZE_JOINT_FREE_FLYER(' +joint.name + ');\n'
        if joint.joint_type == 'revolute':
          laxis = joint.axis.split(' ')
          s=s_width+'INITIALIZE_JOINT_REVOLUTE_AXIS_ANY(' +joint.name 
          s=s+ laxis[0] + ',' + laxis[1] + ',' + laxis[2] + ');\n'                    
        f.write(s)

        # Initialize name
        s = 'const std::string ' + joint.name + '::name = "' + joint.name '";\n'
        f.write(s)

        
    def generate_init_body(self,f,body):

        
    def generate_init(self):
        # Open file
        lfilename = self.urdf.name + '.cc'
        f = open(lfilename,'w')
        # Name of the model
        header_suffix = self.urdf.name    
        s = '/*\n * This file is part of the ' + self.urdf.name + ' robot model.\n'
        f.write(s)
        s = ' * It contains the initialization of all the robot bodies and joints.\n'
        f.write(s)
        s = ' */'
        f.write(s)
        # Open namespaces.
        open_metapod_ns(self,f);

        s = '// Initialization of the robot global constants'
        f.write(s)
        s = 'Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;'      
        f.write(s)

        

        # Close namespaces.
        close_metapod_ns(self,f);

        f.close()

    def generate_node(self,f,body,s_switch, lbool):
        if lbool:
            s= body + ',\n'
            f.write(s)

            # Deal with the joint of the current link/body
            if body in self.urdf.parent_map:
                (joint,parent) = self.urdf.parent_map[body]
                if joint:
                    s= s_switch + joint + ',\n'
            else:
                s_switch = s_switch + '             ' 
                s= s_switch + 'base_joint,\n'
            f.write(s)  
                       
        # Deal with the child of the current link
        if body in self.urdf.child_map:
            for v in self.urdf.child_map[body]:
                ljoint = self.urdf.joints[v[0]]
                if ljoint.joint_type !='fixed':
                    s= s_switch+'Node<'
                    f.write(s);
                    pres_switch=s_switch;
                    s_switch= s_switch+'     '
                    self.generate_node(f,v[1],s_switch, True)
                    s = pres_switch + '>,\n'
                    s_switch=pres_switch
                    f.write(s)
                else:
                    self.generate_node(f,v[1],s_switch, False)     
        
    def generate_robot(self):
        lfilename = self.urdf.name + '_robot.hh'
        f = open(lfilename,'w')
        header_suffix = self.urdf.name + '_ROBOT'   
        self.open_header(f,header_suffix)
        
        # Headers
        s='#include "metapod/tools/common.hh"\n'
        s=s+'#include "'+ self.urdf.name +'_joint.hh"\n'
        s=s+'#include "'+ self.urdf.name +'_body.hh"\n'
        f.write(s)

        self.open_metapod_ns(f)
        self.open_robot_ns(f,self.urdf.name)        
        s_width='    '
        s=s_width + '// Model of the robot. Contains data at the global robot level\n'
        s=s+ s_width + '// and the tree of Body/Joint\n'

        f.write(s)
        
        # Class
        s=s_width + 'class METAPOD_DLLEXPORT ' + self.urdf.name + '\n'
        s=s+s_width + '{\n' 
        f.write(s)
        s=s_width + '  public:'
        f.write(s)

        s_width = s_width+'  '
        s=s_width + '// Global constants or variable of the robot'
        f.write(s)
        s=s_width + 'enum { NBDOF = ' + str(self.nb_dof) + '};\n'
        f.write(s)
        s=s_width + 'static Eigen::Matrix< FloatType, NBDOF, NBDOF> H;\n'
        f.write(s)
        s=s_width + 'static Eigen::Matrix< FloatType, NBDOF, 1> confVector;\n\n'
        f.write(s)
        
        # Generate nodes
        s=s_width + 'typedef Node<'
        f.write(s)
        s=s_width + '             '
        for link in self.urdf.links:
            if link not in self.urdf.parent_map:
              self.generate_node(f, link, s_width, True)

        s=s_width + '>'      
        self.close_robot_ns(f)
        self.close_metapod_ns(f)

        self.close_header(f,header_suffix)
        f.close()
        

    def generate_metapod(self):
        self.generate_bodies()
        self.generate_joints()
        self.generate_robot()
        self.generate_template_robot()
        self.generate_init()
