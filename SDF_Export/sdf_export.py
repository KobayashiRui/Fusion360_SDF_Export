#sdf出力
import adsk.core, adsk.fusion, traceback
import math
import os
import xml.etree.ElementTree as Et
import xml.dom.minidom as md

#4x4の同次変換行列から平行移動vectorを取得する&cmをmに変換
def get_vector(matrix):
    vector = [matrix[3] / 100.0, matrix[7] / 100, matrix[11] / 100]
    return vector

#4x4の行列を引数に入れることで回転行列を抽出する
def get_rotation_matrix(matrix): 
    rotation_matrix =  [[matrix[0],matrix[1],matrix[2]],
                        [matrix[4],matrix[5],matrix[6]], 
                        [matrix[8],matrix[9],matrix[10]]]
    return rotation_matrix

#3x3の行列から逆行列を求める
def inv_matrix(matrix): 
    determinant = (matrix[0][0] * matrix[1][1] * matrix[2][2] + 
                   matrix[0][1] * matrix[1][2] * matrix[2][0] +
                   matrix[0][2] * matrix[1][0] * matrix[2][1] - 
                   matrix[0][2] * matrix[1][1] * matrix[2][0] -
                   matrix[0][1] * matrix[1][0] * matrix[2][2] -
                   matrix[0][0] * matrix[1][2] * matrix[2][1])

    inv_matrix = [[0,0,0],[0,0,0],[0,0,0]]
    inv_matrix[0][0] = (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) / determinant
    inv_matrix[0][1] = -1 * (matrix[0][1] * matrix[2][2] - matrix[0][2]* matrix[2][1])/ determinant
    inv_matrix[0][2] = (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) / determinant
    inv_matrix[1][0] = -1 * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) / determinant
    inv_matrix[1][1] = (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) / determinant
    inv_matrix[1][2] = -1 * (matrix[0][0] * matrix[1][2] - matrix[0][2] * matrix[1][0])/ determinant
    inv_matrix[2][0] = (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) / determinant
    inv_matrix[2][1] = -1 * (matrix[0][0] * matrix[2][1] - matrix[0][1] * matrix[2][0]) / determinant
    inv_matrix[2][2] = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) / determinant

    return inv_matrix

#回転行列からroll,pitch,yawを求める
def rotation_matrix2roll_pitch_yaw(matrix):

    pitch = math.atan2(-1 * matrix[2][0], math.sqrt((matrix[2][1]**2) + (matrix[2][2]**2)))
    yaw   = math.atan2(matrix[1][0], matrix[0][0]) 
    roll  = math.atan2(matrix[2][1], matrix[2][2])

    return [roll,pitch,yaw]

#イナーシャと質量と重心位置から重心周りのイナーシャを求める
def get_com_inertia(inertia,mass,com):
    translation = [com[1]**2+com[2]**2, com[0]**2+com[2]**2, com[0]**2+com[1]**2,
                 -com[0]*com[1], -com[1]*com[2], -com[0]*com[2]]
    com_inertia = [ (i - mass*t)/10000 for i, t in zip(inertia, translation)] 
    return com_inertia

#相対距離を算出
def relative_distance(vector0,vector1): #vector0 - vector1
    relative_distance = [(v0 - v1)   for v0, v1 in zip(vector0, vector1)] 
    return relative_distance

def get_com_vector(com,pos):
    return [ (com_/100.0 - pos_) for com_, pos_ in zip(com,pos)]

def get_joint_link_vector(joint_point,link_point):
    return [ (joint_point_/100.0 - link_point_) for joint_point_, link_point_ in zip(joint_point,link_point)]


def matrix_multiply(matrix, vector):
    result_matrix = [ ((matrix[0][0] * vector[0]) +  (matrix[0][1] * vector[1]) + (matrix[0][2] * vector[2])),
                      ((matrix[1][0] * vector[0]) +  (matrix[1][1] * vector[1]) + (matrix[1][2] * vector[2])),
                      ((matrix[2][0] * vector[0]) +  (matrix[2][1] * vector[1]) + (matrix[2][2] * vector[2]))]
    return result_matrix

def extend_xyz_rpy(xyz,rpy):
    return [xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2]]


class SDF():
    def __init__(self,app):#rootComp.occurrences,rootComp.joints
        self.ui  = app.userInterface
        self.design = adsk.fusion.Design.cast(app.activeProduct)        
        self.joint_type_list = [ 'fixed', 'revolute', 'prismatic', 'Cylinderical','PinSlot', 'Planner', 'Ball']
        self.occurrence_list = self.design.rootComponent.occurrences
        self.joint_list = self.design.rootComponent.joints
        self.Robot_Name = self.design.rootComponent.name.replace(' ', '_')
        self.Link_List = []
        self.Joint_List=[]
        self.base_path = False
        self.robot_path = False

    def Get_Link_Data(self): #リンクデータの取得
        for i in range(self.occurrence_list.count):
            _link_buf = {}
            _occurrence = self.occurrence_list.item(i)

            _link_buf["name"] = _occurrence.name #リンクの名前の取得

            #4*4の変換行列の取得
            _link_matrix = _occurrence.transform.asArray()
            _rotation_matrix =  get_rotation_matrix(_link_matrix) #変換行列から回転行列の取得
            _inv_rotation_matrix = inv_matrix(_rotation_matrix)

            #位置と姿勢データ
            _pose_xyz =  get_vector(_link_matrix) #変換行列から平行移動ベクトルを取得
            _pose_rpy = rotation_matrix2roll_pitch_yaw(_rotation_matrix)
            _link_buf["pose"] = extend_xyz_rpy(_pose_xyz,_pose_rpy)

            #質量,イナーシャの取得
            (_returnValue, _xx, _yy, _zz, _xy, _yz, _xz) = _occurrence.physicalProperties.getXYZMomentsOfInertia()
            _mass = _occurrence.physicalProperties.mass #質量の取得
            _link_buf["mass"]= _mass

            _com = _occurrence.physicalProperties.centerOfMass.asArray()#重心位置
            _com_inertia = get_com_inertia([_xx, _yy, _zz, _xy, _yz, _xz], _mass, _com) #物性質の取得
            _link_buf["inertia"] = [_com_inertia[0],_com_inertia[3],_com_inertia[5],_com_inertia[1],_com_inertia[4],_com_inertia[2]]

            #comの位置と姿勢
            _com_vector = get_com_vector(_com,_pose_xyz)#重心の平行移動
            _com_xyz = matrix_multiply(_inv_rotation_matrix,_com_vector)
            _com_rpy = rotation_matrix2roll_pitch_yaw(_inv_rotation_matrix)
            _link_buf["com"] = extend_xyz_rpy(_com_xyz, _com_rpy)
            self.Link_List.append(_link_buf)
    
    def Get_Joint_Data(self):
        for i in range(self.joint_list.count):
            _joint_buf = {}
            _joint = self.joint_list.item(i)

            #回転関節
            if self.joint_type_list[_joint.jointMotion.jointType] == "revolute":
                _joint_buf["name"] = "rev" + str(i)
                _joint_buf["child"] = _joint.occurrenceOne.fullPathName.split("+")[0]
                _joint_buf["parent"] = _joint.occurrenceTwo.fullPathName.split("+")[0]

                _child_link_matrix = self.Search_Child(_joint_buf["child"])
                _child_xyz_world =  get_joint_link_vector(_joint.geometryOrOriginOne.origin.asArray(), get_vector(_child_link_matrix))  #平行移動
                _child_link_rotation_matrix = inv_matrix(get_rotation_matrix(_child_link_matrix)) #回転行列
                _child_xyz = matrix_multiply(_child_link_rotation_matrix, _child_xyz_world)
                _child_rpy = rotation_matrix2roll_pitch_yaw(_child_link_rotation_matrix)
                _joint_buf["pose"] = extend_xyz_rpy(_child_xyz, _child_rpy)

                #回転軸
                _joint_buf["axis"] = [axis_data for axis_data in _joint.jointMotion.rotationAxisVector.asArray()]
                self.Joint_List.append(_joint_buf)


    def Search_Child(self, child_name):
        for i in range(self.occurrence_list.count):
            if self.occurrence_list.item(i).name == child_name:
                return self.occurrence_list.item(i).transform.asArray()

    def Get_Datas(self):
        self.Get_Link_Data()
        self.Get_Joint_Data()


    def Write_SDF(self):
        self.robot_path = self.base_path + "/" + self.Robot_Name #todo このスクリプトファイルまでの絶対パスを取得する
        #os.mkdir(_robot_path) # todo ファイルがあるか確認する
        os.makedirs(self.robot_path, exist_ok=True)#絶対パスでないとだめ?

        root = Et.Element("sdf", {"version":"1.6"})
        model_el = Et.SubElement(root, "model", {"name":self.Robot_Name})
        model_pose_el = Et.SubElement(model_el, "pose")
        model_pose_el.text = "0 0 0 0 0 0"

        for _link in self.Link_List:
            link_el = Et.SubElement(model_el, "link", {"name": _link["name"]})

            link_pose_el = Et.SubElement(link_el, "pose")
            link_pose_el.text = (' '.join(list(map(str,_link["pose"])) ))
            
            link_inertial_el = Et.SubElement(link_el, "inertial")
            inertial_mass_el = Et.SubElement(link_inertial_el, "mass")
            inertial_mass_el.text = str(_link["mass"])
            inertial_pose_el = Et.SubElement(link_inertial_el, "pose")
            inertial_pose_el.text = (' '.join(list(map(str,_link["com"])) )) 
            inertial_inertia_el = Et.SubElement(link_inertial_el, "inertia")
            inertia_ixx_el = Et.SubElement(inertial_inertia_el, "ixx")
            inertia_ixx_el.text = str(_link["inertia"][0]) 
            inertia_ixy_el = Et.SubElement(inertial_inertia_el, "ixy")
            inertia_ixy_el.text = str(_link["inertia"][1]) 
            inertia_ixz_el = Et.SubElement(inertial_inertia_el, "ixz")
            inertia_ixz_el.text = str(_link["inertia"][2]) 
            inertia_iyy_el = Et.SubElement(inertial_inertia_el, "iyy")
            inertia_iyy_el.text = str(_link["inertia"][3]) 
            inertia_iyz_el = Et.SubElement(inertial_inertia_el, "iyz")
            inertia_iyz_el.text = str(_link["inertia"][4]) 
            inertia_izz_el = Et.SubElement(inertial_inertia_el, "izz")
            inertia_izz_el.text = str(_link["inertia"][5]) 

            link_collision_el = Et.SubElement(link_el, "collision", {"name":_link["name"] + "_collision"})
            collision_geometry_el = Et.SubElement(link_collision_el, "geometry")
            collision_geometry_mesh_el = Et.SubElement(collision_geometry_el, "mesh")
            collision_mesh_uri_el = Et.SubElement(collision_geometry_mesh_el, "uri")
            collision_mesh_uri_el.text = "model://" + self.Robot_Name + "/meshes/" + _link["name"].split(":")[0] + ".stl"

            link_visual_el = Et.SubElement(link_el, "visual" ,{"name":_link["name"] + "_visual"})
            visual_geometry_el = Et.SubElement(link_visual_el, "geometry")
            visual_geometry_mesh_el = Et.SubElement(visual_geometry_el, "mesh")
            visual_mesh_uri_el = Et.SubElement(visual_geometry_mesh_el, "uri")
            visual_mesh_uri_el.text = "model://" + self.Robot_Name + "/meshes/" + _link["name"].split(":")[0] + ".stl"


        for _joint in self.Joint_List:
            joint_el = Et.SubElement(model_el, "joint", {"type":"revolute", "name":_joint["name"]})

            joint_pose_el = Et.SubElement(joint_el, "pose")
            joint_pose_el.text = (' '.join(list(map(str,_joint["pose"])))) 
            joint_child_el = Et.SubElement(joint_el, "child")
            joint_child_el.text =_joint["child"] 
            joint_parent_el = Et.SubElement(joint_el, "parent")
            joint_parent_el.text = _joint["parent"]
            joint_axis_el = Et.SubElement(joint_el, "axis")
            joint_axis_xyz_el = Et.SubElement(joint_axis_el, "xyz")
            joint_axis_xyz_el.text = (' '.join(list(map(str,_joint["axis"]))))
        

        xmlFile = open(self.robot_path + "/model.sdf", "w")
        document = md.parseString(Et.tostring(root, 'utf-8'))
        document.writexml(
            xmlFile,
            encoding = 'utf-8',
            newl = "\n",
            indent = "",
            addindent = "\t"
        )

    def Write_STL(self):
        exportMgr = self.design.exportManager
        _stl_path = self.robot_path+"/meshes"
        os.makedirs(_stl_path, exist_ok=True)
        for i in range(self.occurrence_list.count):
            # create stl exportOptions
            #todo すでに作成したstlファイルは飛ばす
            stlExportOptions = exportMgr.createSTLExportOptions(self.occurrence_list.item(i), _stl_path + "/" + self.occurrence_list.item(i).name.split(":")[0])
            stlExportOptions.sendToPrintUtility = False
            stlExportOptions.isBinaryFormat = True
            # options are .MeshRefinementLow .MeshRefinementMedium .MeshRefinementHigh
            stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            exportMgr.execute(stlExportOptions)


    def Write_SDF_Config(self):
        #sdf configの作成
        with open(self.robot_path + "/model.config", mode="w") as f:
            f.write("<?xml version='1.0'?>\n")
            f.write("<model>\n")
            f.write("<name>" + self.Robot_Name + "</name>\n")
            f.write("<version>1.4</version>\n")
            f.write("<sdf version='1.6'>model.sdf</sdf>\n")
            f.write("<author>\n")
            f.write("<name></name>\n")
            f.write("<email></email>\n")
            f.write("</author>\n")
            f.write("<description></description>\n")
            f.write("</model>\n")

    
    def Set_Export_Path(self):
        ##############path選択画面の表示################################################
        # Set styles of folder dialog.
        folderDlg = self.ui.createFolderDialog()
        folderDlg.title = 'Fusion Folder Dialog' 
    
        # Show folder dialog
        dlgResult = folderDlg.showDialog()
        if dlgResult == adsk.core.DialogResults.DialogOK:
            self.base_path =folderDlg.folder 
        else:
            self.base_path = False
        ###############################################################################


    def Write_Data(self):
        self.Set_Export_Path()
        self.Write_SDF()
        self.Write_STL()
        self.Write_SDF_Config()




            

app = adsk.core.Application.get()
ui  = app.userInterface

#基本的にcm単位なので注意
def run(context):
    try:
        #オカレンスの取得
        sdf = SDF(app)
        sdf.Get_Datas()
        sdf.Write_Data()
        ui.messageBox("Complete")
    except:
        if ui:
            #ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
            ui.messageBox('Error')
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


    

