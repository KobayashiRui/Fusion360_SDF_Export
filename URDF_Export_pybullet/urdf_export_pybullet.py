#urdf出力
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
    return [(v0 - v1)   for v0, v1 in zip(vector0, vector1)]

def relative_distance2m(vector0,vector1): #vector0[cm]/100 - vector1[m]
    return [(v0/100 - v1)   for v0, v1 in zip(vector0, vector1)]

def cm2m(vector0):
    return [v0/100 for v0 in vector0]

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


class URDF():
    def __init__(self,app):#rootComp.occurrences,rootComp.joints
        self.ui  = app.userInterface
        self.design = adsk.fusion.Design.cast(app.activeProduct)        
        self.joint_type_list = [ 'fixed', 'revolute', 'prismatic', 'Cylinderical','PinSlot', 'Planner', 'Ball']
        self.occurrence_list = self.design.rootComponent.occurrences
        self.joint_list = self.design.rootComponent.joints
        self.Robot_Name = self.design.rootComponent.name.replace(' ', '_')
        self.Link_List = {}
        self.Joint_List={}
        self.base_path = False
        self.robot_path = False

    def Get_Link_Data(self): #リンクデータの取得
        for i in range(self.occurrence_list.count):
            _link_buf = {}
            _occurrence = self.occurrence_list.item(i)

            _link_buf["name"] = _occurrence.name.replace(' ', '_') #リンクの名前の取得 (空白を_に)

            #4*4の変換行列の取得
            _link_matrix = _occurrence.transform.asArray()
            _rotation_matrix =  get_rotation_matrix(_link_matrix) #変換行列から回転行列の取得
            _inv_rotation_matrix = inv_matrix(_rotation_matrix)

            #位置と姿勢データ(位置と姿勢は後から変更する)
            #位置データは最初は0,0,0
            #_pose_xyz = get_vector(_link_matrix) #変換行列から平行移動ベクトルを取得
            #_pose_rpy = rotation_matrix2roll_pitch_yaw(_rotation_matrix)
            #_link_buf["pose"] = extend_xyz_rpy(_pose_xyz,_pose_rpy)
            _link_buf["link_matrix"] = _link_matrix
            _link_buf["origin_pos"] = [0,0,0]

            #質量,イナーシャの取得
            (_returnValue, _xx, _yy, _zz, _xy, _yz, _xz) = _occurrence.physicalProperties.getXYZMomentsOfInertia()
            _mass = _occurrence.physicalProperties.mass #質量の取得
            _link_buf["mass"]= _mass

            _com = _occurrence.physicalProperties.centerOfMass.asArray()#重心位置 
            _com_inertia = get_com_inertia([_xx, _yy, _zz, _xy, _yz, _xz], _mass, _com) #物性質の取得
            _link_buf["inertia"] = [_com_inertia[0],_com_inertia[3],_com_inertia[5],_com_inertia[1],_com_inertia[4],_com_inertia[2]]

            #comの位置と姿勢
            #TODO 要修正
            #重心位置(絶対座標とlinkの原点位置との相対座標?)
            #_com_vector = get_com_vector(_com,_pose_xyz)#重心の平行移動
            #_com_xyz = matrix_multiply(_inv_rotation_matrix,_com_vector)
            #_com_rpy = rotation_matrix2roll_pitch_yaw(_inv_rotation_matrix)
            _com_xyz = get_com_vector(_com, [0,0,0])
            _com_rpy = [0,0,0]
            _link_buf["com"] = extend_xyz_rpy(_com_xyz, _com_rpy)
            _link_buf["com_buf"] = _com
            self.Link_List[_link_buf["name"]] = _link_buf
    
    def Get_Joint_Data(self):
        #各ジョイントのoriginを更新する=>重心ジョイントの座標を更新する=>原点座標を更新する
        for i in range(self.joint_list.count):
            #ジョイント原点を設定する
            _joint = self.joint_list.item(i)
            _child_name = _joint.occurrenceOne.fullPathName.split("+")[0].replace(' ', '_')
            self.Link_List[_child_name]["origin_pos"] = cm2m(_joint.geometryOrOriginOne.origin.asArray())
            #重心位置をジョイント原点からの相対にする
            _com = self.Link_List[_child_name]["com_buf"]
            _com_xyz = get_com_vector(_com, self.Link_List[_child_name]["origin_pos"])
            _com_rpy = [0,0,0]
            self.Link_List[_child_name]["com"] = extend_xyz_rpy(_com_xyz, _com_rpy)


        for i in range(self.joint_list.count):
            _joint_buf = {}
            _joint = self.joint_list.item(i)

            #回転関節
            if self.joint_type_list[_joint.jointMotion.jointType] == "revolute":

                #TODO 名前の部分の処理を改善する
                _joint_buf["name"] = "rev" + str(i)

                #TODO 制限が設定されている => revolute, 設定されていない => continuous
                _joint_buf["type"] = "continuous"
                #_joint_buf["type"] = "revolute"
                _joint_buf["child"] = _joint.occurrenceOne.fullPathName.split("+")[0].replace(' ', '_')
                _joint_buf["parent"] = _joint.occurrenceTwo.fullPathName.split("+")[0].replace(' ', '_')

                #parentのorigin_posを取得
                _parent_origin_pos = self.Link_List[_joint_buf["parent"]]["origin_pos"]

                _joint_xyz = relative_distance2m(_joint.geometryOrOriginOne.origin.asArray(),_parent_origin_pos)

                ##childに原点を設定する
                #self.Link_List[_joint_buf["child"]]["origin_pos"] = cm2m(_joint.geometryOrOriginOne.origin.asArray())
                

                #_parent_link_matrix = self.Link_List[_joint_buf["parent"]]["link_matrix"]
                #_parent_link_rotation_matrix = inv_matrix(get_rotation_matrix(_parent_link_matrix)) #回転行列
                _joint_rpy = [0,0,0] #rotation_matrix2roll_pitch_yaw(_parent_link_rotation_matrix)

                _joint_buf["pose"] = extend_xyz_rpy(_joint_xyz, _joint_rpy)

                #回転軸
                _joint_buf["axis"] = [axis_data for axis_data in _joint.jointMotion.rotationAxisVector.asArray()]
                self.Joint_List[_joint_buf["name"]] = _joint_buf
    
    #jointのデータを元にLinkのposeデータを設定していく=>STLファイルの原点を合わせるイメージ
    def Set_Link_Pose_Data(self):
        for key in self.Link_List:
            _link_matrix = self.Link_List[key]["link_matrix"]
            _pose_xyz_world = get_vector(_link_matrix) #変換行列から平行移動ベクトルを取得
            _rotation_matrix =  get_rotation_matrix(_link_matrix) #変換行列から回転行列の取得
            _pose_rpy = rotation_matrix2roll_pitch_yaw(_rotation_matrix) 
            _origin_pos_world = self.Link_List[key]["origin_pos"]
            _pose_xyz = relative_distance(_pose_xyz_world, _origin_pos_world)
            self.Link_List[key]["pose"] = extend_xyz_rpy(_pose_xyz,_pose_rpy)




    def Search_Child(self, child_name):
        for i in range(self.occurrence_list.count):
            if self.occurrence_list.item(i).name == child_name:
                return self.occurrence_list.item(i).transform.asArray()

    def Search_Parent(self, parent_name):
        for i in range(self.occurrence_list.count):
            if self.occurrence_list.item(i).name == parent_name:
                return self.occurrence_list.item(i).transform.asArray()

    def Get_Datas(self):
        self.Get_Link_Data()
        self.Get_Joint_Data()
        self.Set_Link_Pose_Data()


    def Write_URDF(self):
        self.robot_path = self.base_path + "/" + self.Robot_Name #todo このスクリプトファイルまでの絶対パスを取得する
        #os.mkdir(_robot_path) # todo ファイルがあるか確認する
        os.makedirs(self.robot_path, exist_ok=True)#絶対パスでないとだめ?

        #Make XML
        root = Et.Element("robot",{"name":self.Robot_Name})

        #Make Link XML
        for _link in self.Link_List.values():
            link_el = Et.SubElement(root, "link", {"name":_link["name"]})

            inertial_el = Et.SubElement(link_el, "inertial")
            inertial_mass_el = Et.SubElement(inertial_el, "mass", {"value":str(_link["mass"])})
            inertial_origin_el = Et.SubElement(inertial_el, "origin", {"rpy":(' '.join(list(map(str,_link["com"][3:])))), "xyz":(' '.join(list(map(str,_link["com"][:3])))) })
            inertial_inertia_el = Et.SubElement(inertial_el, "inertia", {"ixx":str(_link["inertia"][0]), "ixy":str(_link["inertia"][1]), "ixz":str(_link["inertia"][2]), "iyy":str(_link["inertia"][3]), "iyz":str(_link["inertia"][4]), "izz":str(_link["inertia"][5])})

            collision_el = Et.SubElement(link_el, "collision", {"name":_link["name"]+"_collision"})
            collision_origin_el = Et.SubElement(collision_el, "origin", {"rpy":(' '.join(list(map(str,_link["pose"][3:])))), "xyz":(' '.join(list(map(str,_link["pose"][:3]))))})
            collision_geometry_el = Et.SubElement(collision_el, "geometry")
            collision_geometry_mesh_el = Et.SubElement(collision_geometry_el, "mesh", {"filename":"./meshes/" + _link["name"].split(":")[0].replace(' ', '_') + ".stl", "scale":"1.0 1.0 1.0"})

            visual_el = Et.SubElement(link_el, "visual", {"name":_link["name"] + "_visual"})
            visual_origin_el = Et.SubElement(visual_el, "origin", {"rpy":(' '.join(list(map(str,_link["pose"][3:])))), "xyz":(' '.join(list(map(str,_link["pose"][:3]))))})
            visual_geometry_el = Et.SubElement(visual_el, "geometry")
            visual_geometry_mesh_el = Et.SubElement(visual_geometry_el, "mesh", {"filename":"./meshes/" + _link["name"].split(":")[0].replace(' ', '_') + ".stl", "scale":"1.0 1.0 1.0"})
        
        #Make Joint XML
        for _joint in self.Joint_List.values(): 
            joint_el = Et.SubElement(root, "joint", {"name":_joint["name"], "type":_joint["type"]})
            if _joint["type"] == "revolute":
                parent_el = Et.SubElement(joint_el, "parent", {"link":_joint["parent"]})
                child_el = Et.SubElement(joint_el, "child", {"link":_joint["child"]})
                joint_origin_el = Et.SubElement(joint_el, "origin", {"rpy":(' '.join(list(map(str,_joint["pose"][3:])))), "xyz":(' '.join(list(map(str,_joint["pose"][:3]))))})
                axis_el = Et.SubElement(joint_el, "axis", {"xyz":(' '.join(list(map(str,_joint["axis"]))))})
                limit_el = Et.SubElement(joint_el, "limit", {"lower":"-3.14", "upper":"3.14", "effort":"0", "velocity":"0"})

            elif _joint["type"] == "continuous":
                parent_el = Et.SubElement(joint_el, "parent", {"link":_joint["parent"]})
                child_el = Et.SubElement(joint_el, "child", {"link":_joint["child"]})
                joint_origin_el = Et.SubElement(joint_el, "origin", {"rpy":(' '.join(list(map(str,_joint["pose"][3:])))), "xyz":(' '.join(list(map(str,_joint["pose"][:3]))))})
                axis_el = Et.SubElement(joint_el, "axis", {"xyz":(' '.join(list(map(str,_joint["axis"]))))})
    
        xmlFile = open(self.robot_path + "/model.urdf", "w")
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
            stlExportOptions = exportMgr.createSTLExportOptions(self.occurrence_list.item(i), _stl_path + "/" + self.occurrence_list.item(i).name.split(":")[0].replace(' ', '_'))
            stlExportOptions.sendToPrintUtility = False
            stlExportOptions.isBinaryFormat = True
            # options are .MeshRefinementLow .MeshRefinementMedium .MeshRefinementHigh
            stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            exportMgr.execute(stlExportOptions)
    
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
        self.Write_URDF()
        self.Write_STL()




            

app = adsk.core.Application.get()
ui  = app.userInterface

#基本的にcm単位なので注意
def run(context):
    try:
        #オカレンスの取得
        urdf = URDF(app)
        urdf.Get_Datas()
        urdf.Write_Data()
        ui.messageBox("Complete")
    except:
        if ui:
            #ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
            ui.messageBox('Error')
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


    

