import roslib; roslib.load_manifest('urdf_parser_py')
import rospy
import sys
from tf.transformations import euler_matrix
#import euler_from_quaternion
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
        if self.urdf.joints[joint].joint_type != 'fixed':
          if body_parent is None:
            s= s + ', 0, NP, base_joint );\n'
          else:
            s= s + ', 1,' + body_parent 
            s= s + ', ' + joint + ');\n'
          f.write(s)
        else:
          if body_parent=='base_link':
            s= s + ', 0 , NP, ' + joint + ' );\n'  
            f.write(s)
        
        if body in self.urdf.child_map:
          for v in self.urdf.child_map[body]:
            joint = v[0]
            lchild = v[1]
            if self.urdf.joints[joint].joint_type=='fixed':
              if body_parent is not None:
                  body=body_parent
              else:
                  body='base_link'
            self.generate_body(f, lchild, joint, body)
            
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
              v = self.urdf.child_map[k]
              joint = v[0][0]
              self.generate_body(f, k, joint)

        self.close_robot_ns(f)
        self.close_metapod_ns(f)

        self.close_header(f,header_suffix)
        f.close()

    # Bodies related method
    def generate_joint(self,f,joint):
        towrite=False
        s = '      ';
        # Detect root
        rootparent=False
        if joint.parent not in self.urdf.parent_map:
          rootparent=True

        if (joint.joint_type == 'floating') | rootparent:
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

    def generate_template_rnea_crba_robot(self,f,lextern=False):
        s_prefix=''
        if lextern:
            s_prefix='extern';
        s=s_prefix + ' template struct metapod::crba<metapod::'+ self.urdf.name + '::Robot,true>;\n'
        f.write(s)
        s=s_prefix + ' template struct metapod::rnea<metapod::'+ self.urdf.name + '::Robot,true>;\n'
        f.write(s)      
        s=s_prefix + ' template struct metapod::crba<metapod::'+ self.urdf.name + '::Robot,false>;\n'
        f.write(s)      
        s=s_prefix + ' template struct metapod::rnea<metapod::'+ self.urdf.name + '::Robot,false>;\n\n'
        f.write(s)

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
        s ='#include "' + self.urdf.name + '_robot.hh"\n\n'
        f.write(s)
        self.generate_template_rnea_crba_robot(f,True)

        self.close_header(f,header_suffix)
        f.close()

    def generate_init_joint(self,f,joint,label):
        s_width='  ';
        okToWrite = False;

        # Initialize joint
        if joint.joint_type == 'floating':
          s=s_width+'INITIALIZE_JOINT_FREE_FLYER(' +joint.name + ');\n'
          okToWrite = True;

        if joint.joint_type == 'revolute':
          laxis = joint.axis.split(' ')
          s=s_width+'INITIALIZE_JOINT_REVOLUTE_AXIS_ANY(' +joint.name +','
          s=s+ laxis[0] + ',' + laxis[1] + ',' + laxis[2] + ');\n'                    
          okToWrite = True;
        
        if not okToWrite:
          return label
        
        s2=s_width+ '// Initialization of '+joint.name +'\n'
        f.write(s2)        
        f.write(s)
        
        # Initialize name
        s=s_width + 'const std::string ' + joint.name + '::name = "' + joint.name + '";\n'
        f.write(s)
        
        # Write label
        s=s_width + 'const int ' + joint.name + '::label = ' + str(label) + ';\n'
        f.write(s)

        # Position in configuration
        s=s_width + 'const int ' + joint.name + '::positionInConf = ' + str(label-1) + ';\n'
        f.write(s)
        
        # Transform of the joint
        s=s_width + 'const Spatial::Transform ' + joint.name + '::Xt = Spatial::Transform(\n'
        f.write(s)
        alpha = joint.origin.rotation[0]
        beta = joint.origin.rotation[1]
        gamma = joint.origin.rotation[2]
        Re = euler_matrix(alpha, beta, gamma, 'rxyz')
        s=s_width + '  matrix3dMaker('
        s=s+  str(Re[0][0]) + ',' + str(Re[0][1]) + ',' + str(Re[0][2]) + ',\n'
        f.write(s)
        s=s_width + '   ' + str(Re[1][0]) + ',' + str(Re[1][1]) + ',' + str(Re[1][2]) + ',\n'
        f.write(s)
        s=s_width + '   ' + str(Re[2][0]) + ',' + str(Re[2][1]) + ',' + str(Re[2][2]) + '),\n'
        f.write(s)
        s=s_width + '  vector3d('+ str(joint.origin.position[0])+','\
          + str(joint.origin.position[1])+','\
          + str(joint.origin.position[2])+'));\n'
        f.write(s)
        return label+1
        
    def generate_init_body(self,f,body,label):
        if body.inertial == None:
          return label

        # initialize link
        s_width='  '
        s=s_width + 'INITIALIZE_BODY(' + body.name + ');\n'
        f.write(s)                
        s=s_width + '// Initialization of '+body.name + '\n'
        f.write(s)
        
        # Set name
        s=s_width + 'const std::string ' + body.name + '::name = "' + body.name + '";\n'
        f.write(s)
        s=s_width + 'const int ' + body.name + '::label = ' + str(label) + ';\n'
        f.write(s)
        s=s_width + 'const FloatType ' + body.name + '::mass = ' + str(body.inertial.mass) + ';\n'
        f.write(s)
        
        s=s_width + 'const vector3d ' + body.name + '::CoM = vector3d('
        s=s+ str(body.inertial.origin.position[0]) + ',' +\
             str(body.inertial.origin.position[1]) + ',' +\
             str(body.inertial.origin.position[2]) + ');\n'

        f.write(s)
        
        s=s_width + 'const matrix3d '+ body.name + '::inertie = matrix3dMaker(\n'
        s=s+s_width + '  ' +str(body.inertial.matrix['ixx']) + ',' +\
                            str(body.inertial.matrix['ixy']) + ',' +\
                            str(body.inertial.matrix['ixz']) + ',\n'

        s=s+s_width + '  ' +str(body.inertial.matrix['ixy']) + ',' +\
                            str(body.inertial.matrix['iyy']) + ',' +\
                            str(body.inertial.matrix['iyz']) + ',\n'

        s=s+s_width + '  ' +str(body.inertial.matrix['ixz']) + ',' +\
                            str(body.inertial.matrix['iyz']) + ',' +\
                            str(body.inertial.matrix['izz']) + ');\n'
        f.write(s)
        
        s=  s_width + 'Spatial::Inertia ' + body.name + '::I = spatialInertiaMaker(' + body.name + '::mass,\n'
        s=s+s_width + '                 ' +             '                          ' + body.name + '::CoM,\n'
        s=s+s_width + '                 ' +             '                          ' + body.name + '::inertie);\n'
        f.write(s)
        return label+1

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

        s = '# include "' + self.urdf.name + '.hh"\n'
        f.write(s)
        
        self.generate_template_rnea_crba_robot(f,False)
        
        # Open namespaces.
        self.open_metapod_ns(f);
        self.open_robot_ns(f,self.urdf.name)

        s = '  // Initialization of the robot global constants\n'
        f.write(s)
        s = '  Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;\n\n'      
        f.write(s)

        # For each joint
        i=0
        for k,v in self.urdf.joints.iteritems():
          i=self.generate_init_joint(f,v,i)

        # For each body
        i=0
        for k,v in self.urdf.links.iteritems():
          i=self.generate_init_body(f,v,i)

        # Close namespaces.
        self.close_robot_ns(f)
        self.close_metapod_ns(f);

        f.close()

    def generate_node(self,f,body,s_switch, lbool, child_left):

        child_size = 0
        # Number of child for the current body
        if body in self.urdf.child_map:
            child_size = len(self.urdf.child_map[body])

        # Get the joint and the father link of current body
        if body in self.urdf.parent_map:
            (joint,parent) = self.urdf.parent_map[body]

        # If the body is to be written
        if lbool:
            s= body
            # Add a comma if there was a parent
            if body in self.urdf.parent_map:
                s+=','
            s+='\n'
            f.write(s)

            # Add the name joint of the current link/body it is not the root.
            if body in self.urdf.parent_map:
                if joint:
                    s= s_switch + joint 
            # If it is not then add base_joint
            else:
                s_switch = s_switch + '             ' 
                s= s_switch + 'base_joint'
            f.write(s)  
                       
        # Deal with the child of the current link
        if body in self.urdf.child_map:
            # We will count the number of Node to be written or not
            generateNodeFalseNb=0
            generateNodeTrueNb=0

            # Going through all the childs
            child_size = len(self.urdf.child_map[body])
            for idx, v in enumerate(self.urdf.child_map[body]):
                child_left = child_size -idx-1
                ljoint = self.urdf.joints[v[0]]

                generateNode=False
                # If the joint is fixed then DO NOT write the body
                if ljoint.joint_type !='fixed':
                    generateNode=True
                else:
                  if body not in self.urdf.parent_map:
                      generateNode=True
                
                if generateNode:        

                    s=''
                    # Add a comma if this is not the root
                    if body in self.urdf.parent_map:    
                        s= ','
                    # Break the line in any case
                    s+='\n'
                    f.write(s)
                    
                    # Add a node corresponding to the child
                    generateNodeTrueNb+=1
                    s= s_switch+'Node<'
                    f.write(s);
                    pres_switch=s_switch;
                    s_switch= s_switch+'     '
                    self.generate_node(f,v[1],s_switch, True,child_left)
                    s = pres_switch + '>'
                    s+='\n'
                    s_switch=pres_switch
                    f.write(s)
                else:
                    # Do n
                    if generateNodeFalseNb==0:
                        s= '\n'     
                        f.write(s)
                    self.generate_node(f,v[1],s_switch, False,child_left)     
                    generateNodeFalseNb+=1
        
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
        s=s_width + 'class METAPOD_DLLEXPORT Robot\n'
        s=s+s_width + '{\n' 
        f.write(s)
        s=s_width + '  public:\n'
        f.write(s)

        s_width = s_width+'  '
        s=s_width + '// Global constants or variable of the robot\n'
        f.write(s)
        s=s_width + 'enum { NBDOF = ' + str(self.nb_dof) + '};\n'
        f.write(s)
        s=s_width + 'static Eigen::Matrix< FloatType, NBDOF, NBDOF> H;\n'
        f.write(s)
        s=s_width + 'typedef Eigen::Matrix< FloatType, NBDOF, 1> confVector;\n\n'
        f.write(s)
        
        # Generate nodes
        s=s_width + 'typedef '
        f.write(s)
        s=s_width + '             '
        for link in self.urdf.links:
            if link not in self.urdf.parent_map:
              self.generate_node(f, link, s_width, False,1)

        s=s_width + ' Tree;\n'      
        s+='    };\n'
        f.write(s)
        self.close_robot_ns(f)
        self.close_metapod_ns(f)

        self.close_header(f,header_suffix)
        f.close()
        
    def generate_test_program(self):
        lfilename = 'test_'+ self.urdf.name + '.cc'
        f = open(lfilename,'w') 
        s=['// Common includes\n# include <string>\n# include <iostream>\n# include <fstream>\n\n',
           '// metapod includes\n# include "' + self.urdf.name +'_robot.hh"\n',
           '# include "' + self.urdf.name + '.hh"\n',
           '# include "metapod/tools/print.hh"\n# include "metapod/tools/initconf.hh"\n\n',
           'using namespace metapod;\nusing namespace ' + self.urdf.name + ';\n',
           'int main(void)\n{\n',
           '  // Set configuration vectors (q, dq, ddq) to reference values.\n',
           '  Robot::confVector q, dq, ddq;\n',
           '  for(int i=0;i< Robot::NBDOF;i++)\n  {\n',
           '    q(i) = dq(i) = ddq(i) = 0.0;\n',
           '  }\n\n',
           '  // Apply the RNEA to the metapod multibody and print the result in a log file.\n',
           '  rnea< Robot, true >::run(q, dq, ddq);\n',
           '  const char result_file[] = "rnea.log";\n',
           '  std::ofstream log(result_file, std::ofstream::out);\n',
           '  printTorques< Robot::Tree>(log);\n',
           '  log.close();\n',
           '}\n']

        for k in s:
            f.write(k)

        f.close()

    def generate_metapod(self):
        self.generate_bodies()
        self.generate_joints()
        self.generate_robot()
        self.generate_template_robot()
        self.generate_init()
        self.generate_test_program()
