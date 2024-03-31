from __future__ import annotations
import numpy as np
import rowan
from ch.bfh.roboticsLab import Base_pb2 as pbBase


class TransformationMatix:
  """!
  Class that handles transformation matrix operations
  Example with robotClient
  client = RobotClient('localhost')
  pose = client.getRobotPose()
  T_BT = TransformationMatrix.fromPose(pose)
  """

  def __init__(self, transformationMatrix: np.ndarray = np.eye(4, dtype=float)) -> None:
    """! Creates a new Transformation matrix
    """
    self.T = transformationMatrix

  @classmethod
  def fromPose(cls, pose: pbBase.Pose)-> TransformationMatix:
    """! 
    Creates a new Transformation matrix from a base.pb.pose pose object
    @param pose: protobuf base.Pose object
    @return TransformationMatrix object
    """
    position = np.reshape(
        [pose.position.x, pose.position.y, pose.position.z], (3, ))
    quaternion = rowan.normalize(
        [pose.orientation.qw, pose.orientation.qx, pose.orientation.qy, pose.orientation.qz])
    rotMatrix = rowan.to_matrix(quaternion)
    tMatrix = np.eye(4, dtype=np.float64)
    tMatrix[:3, :3] = rotMatrix
    tMatrix[:3, -1] = position
    return cls(tMatrix)

  @classmethod
  def compose(cls, position: np.ndarray | list[float], orientation: np.ndarray | list[float])-> TransformationMatix:
    """! Creates TransformationMatrix from position and orientation
    @param position: ndarray or list as [x,y,z] in m
    @param orientation: quaternion defined as [qw,qx,qy,qz]
    @return TransformationMatrix object
    """
    position = np.reshape(position, (3, ))
    quaternion = rowan.normalize(orientation)
    rotMatrix = rowan.to_matrix(quaternion)
    tMatrix = np.eye(4, dtype=np.float64)
    tMatrix[:3, :3] = rotMatrix
    tMatrix[:3, -1] = position
    return cls(tMatrix)

  def pose(self) -> pbBase.Pose:
    '''! Creates a pose object as protobuf message.
    @return: proto base.Pose object
    '''
    try:
      rotMatrix = self.T[:3, :3]
      position = self.T[:3, -1]
      protoPosition = pbBase.Position(x=position[0], y=position[1], z=position[2])
      #TODO: Verify that this conversion works as expected
      # if not go to euler and then to quaternion
      quaternion = rowan.from_matrix(rotMatrix)
      protoQuaternion = pbBase.Quaternion(qw=quaternion[0],qx=quaternion[1],qy=quaternion[2],qz=quaternion[3])
      return pbBase.Pose(position=protoPosition, orientation=protoQuaternion)
    except Exception as e:
      raise Exception('Requester.pose: ', e)
  
  def position(self) -> pbBase.Position:
    '''! Creates a position object as protobuf message.
    @return: proto base.Position object
    '''
    position = self.T[:3, -1]
    return pbBase.Position(x=position[0], y=position[1], z=position[2])

  def orientation(self) -> pbBase.Quaternion:
    '''! Creates a orientation object as protobuf message.
    @return: proto base.Quaternion object
    '''
    rotMatrix = self.T[:3, :3]
    #TODO: Verify that this conversion works as expected
    # if not go to euler and then to quaternion
    quaternion = rowan.from_matrix(rotMatrix)
    return pbBase.Quaternion(qw=quaternion[0],qx=quaternion[1],qy=quaternion[2],qz=quaternion[3])

  def decompose(self) -> tuple[pbBase.Position, pbBase.Quaternion]:
    """! Decompose the transformation matrix in position and orientation
    @return: tuple[pbBase.Position, pbBase.Quaternion]
    """
    return self.position(), self.orientation()

  def decomposeNumpy(self) -> tuple[np.ndarray, np.ndarray]:
    """! Decompose the transformation matrix in position and orientation as numpy arrays
    @return: tuple(np.ndarray, np.ndarray)
    """
    position = np.reshape(self.T[:3, -1],(3,))
    rotMatrix = self.T[:3, :3]
    #TODO: Verify that this conversion works as expected
    # if not go to euler and then to quaternion
    quaternion = rowan.from_matrix(rotMatrix)
    return position, quaternion

  def _inverse(self) -> TransformationMatix:
    """! Transformation matrix inverse
    TODO Optimize the inverse method for homogeneous matrix
    @return: new TransformationMatrix object containing the inverted matrix
    """
    T_inv = np.linalg.inv(self.T)
    return TransformationMatix(T_inv)

  def inv(self) -> TransformationMatix:
    """! Invert the matrix
    @return: new TransformationMatrix object with the inverse
    """
    return self._inverse()

  def __mul__(self, other : TransformationMatix)-> TransformationMatix:
    """! Perform chained TransformationMatrix multiplication
    @usage T_AC = T_AB * T_BC
    @return New TransformationMatrix object
    """
    T = np.matmul(self.T, other.T)
    return TransformationMatix(T)

  def __imul__(self, other : TransformationMatix):
    """! Perform chained TransformationMatrix multiplication
    Overwrites the transformationMatrix in T
    @usage T *= T_1
    """
    self.transformationMatrix = np.matmul(self.T, other.T)

  def __truediv__(self, other: TransformationMatix) -> TransformationMatix:
    """! Perform chained TransformationMatrix multiplication inverting the second matrix
    @usage T_AC = T_AB / T_CB
    @return New TransformationMatrix object
    """
    return self * other.inv()

  def __invert__(self) -> TransformationMatix:
    """! Overloads the not operator ~
    @usage T_AC = T_AB * (~T_CB)
    @return New TransformationMatrix object
    """
    return self._inverse()

  def __eq__(self, other: TransformationMatix) -> bool:
    return np.allclose(self.T, other.T)

  @staticmethod
  def genPosition(position: np.ndarray | list[float]) -> pbBase.Position:
    """! convert a list to a protobuf position
    @param: position as [x,y,z] in m
    @return: Protobuf position object
    """
    return pbBase.Position(x=position[0], y=position[1], z=position[2])

  @staticmethod
  def genOrientation(orientation: np.ndarray | list[float]) -> pbBase.Quaternion:
    """! convert a list to a protobuf Quaternion
    @param orientation: quaternion defined as [qw,qx,qy,qz]
    @return: Protobuf position object
    """
    quaternion = rowan.normalize(orientation)
    return pbBase.Quaternion(qw=quaternion[0],qx=quaternion[1],qy=quaternion[2],qz=quaternion[3])

  def transform(self, vec:np.ndarray | list[float]) -> TransformationMatix:
    """
    Transforms a position [x,y,z] in the transformation matrix frame
    @return: new TransformationMatrix object
    """
    vec = np.array(vec)
    vec = np.reshape(np.append(vec,1),(4,))
    position = np.matmul(self.T, vec)
    newT = self.T.copy()
    newT[:3,-1] = position[:-1]
    return TransformationMatix(newT)

  def __str__(self) -> str:
    """! Display the object position, orientation and matrix
    """
    position, orientation = self.decompose()
    positionStr = "Position: \n\tx: {x:.5f} \n\ty: {y:.5f} \n\tz: {z:.5f}\n".format(x = position.x, y = position.y, z = position.z)
    orientationStr = "Orientation: \n\tqw: {qw:.5f} \n\tqx: {qx:.5f} \n\tqy: {qy:.5f} \n\tqz: {qz:.5f}\n".format(qw = orientation.qw, qx = orientation.qx, qy = orientation.qy, qz = orientation.qz)

    s = [[str(e) for e in row] for row in self.T]
    lens = [max(map(len, col)) for col in zip(*s)]
    fmt = '\t'.join('{{:{}}}'.format(x) for x in lens)
    table = [fmt.format(*row) for row in s]
    tableStr = 'Matrix:\n'+'\n'.join(table)
    return  positionStr+orientationStr+tableStr


#########################################
# Test & Example
#########################################    
if __name__ == '__main__':
  T_B0 = TransformationMatix()
  print(T_B0)
  pose = pbBase.Pose(position=pbBase.Position(x=0.1,y=0.2,z=0.3),orientation=pbBase.Quaternion(qw=1))

  T_01 = TransformationMatix.fromPose(pose)
  print(T_01)

  T_B1 = T_B0*T_01
  print(T_01)

  T_1B = ~T_01 * ~T_B0
  print(T_1B)

  print(T_1B == ~T_B1)
  print(T_1B == T_B1.inv())

  print(T_01.pose())
  P_0 = [0.1,0.2,0.3]
  T_BP = T_B0.transform(P_0)
  print(T_BP.decompose())

  T_10 = ~T_01
  print(T_B1 == T_B0/T_10)
  print(T_B1 == T_B0*T_10.inv())
  print(T_B1.decomposeNumpy())
  print(T_B1.orientation())