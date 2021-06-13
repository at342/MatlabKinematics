# MatlabKinematics
## mDistRedundantInverseKinematics6DofArticulatedSerialArm.m 
- 6関節以上の冗長多関節マニピュレータの逆運動学（位置・姿勢）を解くスクリプト
- 実行結果の例

![7DofIkPosture](https://user-images.githubusercontent.com/53433143/121796675-efa84180-cc55-11eb-825a-4bccc73e6e72.jpg)
![20DofIkPosture](https://user-images.githubusercontent.com/53433143/121796682-f767e600-cc55-11eb-9749-3292e854f3d8.jpg)

### 実行に必要なもの
- Matlab
- Computer Vision Toolbox( rotationVectorToMatrix関数を回転ベクトルの計算に使っているため)
  
### 使い方，7関節の場合

#### 機構パラメータの設定
- 根本側から順番にリンクの長さを縦ベクトルにしてLinkLengthにいれる．
```
LinkLength=[...%リンク長さ
            1;
            1;
            1;
            1;
            1;
            1;
            1;
            ];
```
- 関節の向きの設定
絶対座標のx軸の向きにアーム体幹を伸ばした状態で，x軸回りの関節をRoll関節，y軸，z軸回りの関節をそれぞれPitch, Yaw関節とする．
JointConfigに根本から順番にRoll, Pitch, Yawを関節数だけ横に並べる．
```
JointConfig=[Pitch Yaw Roll Pitch Roll Pitch Yaw];%関節の配置
```
#### 逆運動学の手先目標（位置，姿勢）を設定
- 位置の目標値はPosTargetに縦ベクトルで入れる．
- 姿勢の目標値の設定は回転軸と回転量で指定する．絶対座標から見て目標姿勢への回転軸と平行なベクトルをrotTargetAxisに，回転量[rad]をrotTargetAngleに入れる．
- ちなみに関節角の変数はスクリプト内ではthetaを使っている．
```
%%% 逆運動学の目標値を設定
PosTarget=[rand(1,1)*1.5+3; (rand(1,1)-0.5)*2;(rand(1,1)-0.5)*2];%目標位置, x y z [m]
rotTargetAxis=[(rand(3,1)-0.5)];%目標姿勢への回転軸，縦ベクトル[x;y;z]
rotTargetAngle=(rand(1,1)-0.5)*pi;%目標姿勢への回転量
```

#### 冗長な自由度を決定する最適化の評価関数
関節角thetaPをパラメータとしたスカラー関数なら何でもよい．
この関数Phiを最大化するように冗長自由度が更新される．
例では4関節目のy座標を最大化，つまり肘関節をできるだけ右に持っていくように逆運動学解が更新される．
他の評価関数の候補としては，障害物からのポテンシャル最小化で障害物回避や，関節トルクの２乗ノルムを最小化で消費エネルギ低減などがある．
```
function OptVal = Phi(thetaP,LengthLink,JointConfig,posJoint,frameJoint)%最大化されるスカラー関数Phi
    [posJointP,frameJoinP]=FwardKinematicsArticulatedSerialArm(thetaP,LengthLink,JointConfig);
%     OptVal=-sum( (posJointP(:,4)).^2 );%4関節目の位置の二乗和を最小化
%     OptVal=-posJointP(3,4);%4関節目のz座標を最小化
    OptVal=-posJointP(2,4);%4関節目のy座標を最大化
end
```

### 参考文献
1. 吉川 恒夫, 冗長性を有するロボットの制御, 日本ロボット学会誌, 1984, 2 巻, 6 号, p. 587-592, 公開日 2010/08/25, Online ISSN 1884-7145, Print ISSN 0289-1824, https://doi.org/10.7210/jrsj.2.587, https://www.jstage.jst.go.jp/article/jrsj1983/2/6/2_6_587/_article/-char/ja. 
