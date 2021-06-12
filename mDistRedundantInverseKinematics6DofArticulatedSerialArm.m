%%%シリアルマニピュレータ，回転関節のみ，関節と体幹のオフセット無しの場合のみ

clc
clear
close all

%%%%アームの機構Parameter定義
Roll=[1;0;0];
Pitch=[0;1;0];
Yaw=[0;0;1];

LinkLength=[...%リンク長さ
            1;
            1;
            1;
            1;
            1;
            1;
            1;
            ];
JointConfig=[Pitch Yaw Roll Pitch Roll Pitch Yaw];%関節の配置

if size(LinkLength,1)~=size(JointConfig,2)
    error('uaaaaaaaaaaa')
end
NumJoint=size(LinkLength,1);%関節数

%%%Posture描画用Fig 1
fig=figure(1);
fig.Units='centimeter';
fig.Position=[1 1 10 10];

plBase=fill3([0;-1;-1;0],[0;0;0;0],[0;0;-1;-1],[1 1 1]*6/7);%土台を四角で描画
hold on
grid on
box on
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
xL=[-1 sum(LinkLength)];
yL=[-sum(LinkLength) sum(LinkLength)];
zL=[-sum(LinkLength) sum(LinkLength)];
xlim(xL);
ylim(yL);
zlim(zL);
xticks(xL(1):round( (xL(2)-xL(1) )/4):xL(2));
yticks(yL(1):round( (yL(2)-xL(1) )/4):yL(2));
zticks(zL(1):round( (zL(2)-xL(1) )/4):zL(2));
set(gca,'FontName','Times New Roman','FontSize',10);
view([-1 -0.3 1.8])


%%% 逆運動学の目標値を設定
PosTarget=[rand(1,1)*1.5+3; (rand(1,1)-0.5)*2;(rand(1,1)-0.5)*2];%目標位置, x y z [m]
rotTargetAxis=[(rand(3,1)-0.5)];%目標姿勢への回転軸，縦ベクトル[x;y;z]
rotTargetAngle=(rand(1,1)-0.5)*pi;%目標姿勢への回転量

rotVecTarget=rotTargetAxis/norm(rotTargetAxis)*rotTargetAngle;%目標姿勢への回転ベクトル
rotmTarget=rotationVectorToMatrix(rotVecTarget)';%CV toolboxの座標変換が機械工学の座標変換と転置の関係にあるため転置

plTarget=plot3(PosTarget(1),PosTarget(2),PosTarget(3),'*m','MarkerSize',10);%目標位置描画
%目標姿勢描画用の変数
TipOriX=PosTarget+rotmTarget(:,1);
TipOriY=PosTarget+rotmTarget(:,2);
TipOriZ=PosTarget+rotmTarget(:,3);
%目標姿勢描画
plot3([PosTarget(1), TipOriX(1)],[PosTarget(2) TipOriX(2)],[PosTarget(3) TipOriX(3)],'-r');
plot3([PosTarget(1), TipOriY(1)],[PosTarget(2) TipOriY(2)],[PosTarget(3) TipOriY(3)],'-g');
plot3([PosTarget(1), TipOriZ(1)],[PosTarget(2) TipOriZ(2)],[PosTarget(3) TipOriZ(3)],'-b');

%%%%%先端を目標に合わせる逆運動学, 冗長自由度最適化無し，関節角２乗ノルム最小化
Loop=100;%繰り返し計算上限値
thetaNow=(rand(NumJoint,1)-0.5)*pi/2;%IK収束計算の初期値はランダム
[posJointNow,frameJointNow]=FwardKinematicsArticulatedSerialArm(thetaNow,LinkLength,JointConfig);%順運動学計算
rotVecNow=rotationMatrixToVector(frameJointNow(:,:,end)')';%CV toolboxの関数が機械工学の座標変換と転置の関係にあるため，回転行列を転置してからrotationMatrixToVectorにいれる
IkError=NaN(6,1);%IK誤差
LogError=NaN(Loop,1);%IK誤差のログ用変数

lambda=0.5;%収束計算のスケール変数
for k=1:Loop
    J=ComputeJacobian(posJointNow,frameJointNow,JointConfig);%ヤコビ行列計算
    
    %IK誤差を更新
    IkError(1:3,1)=(PosTarget-posJointNow(:,end) );
    IkError(4:6,1)=rotVecTarget-rotVecNow;

    LogError(k)=sqrt(sse(IkError));
    

    if LogError(k) < 0.005%収束判定
%         'converge'
         
        break
    elseif  LogError(k) < 0.1%誤差のSSEに応じてスケール変数変更
        lambda=0.1;
    elseif  LogError(k) < 1
        lambda = 0.1;        
    end
    
    thetaNow=thetaNow+pinv(J)*IkError*lambda;%関節角を更新
    [posJointNow,frameJointNow]=FwardKinematicsArticulatedSerialArm(thetaNow,LinkLength,JointConfig);%順運動学更新
    
    rotVecNow=rotationMatrixToVector(frameJointNow(:,:,end)')';%回転ベクトル更新
    %CV toolboxの関数が機械工学の座標変換と転置の関係にあるため，回転行列を転置してからrotationMatrixToVectorにいれる

    
end

plIk=plot3(posJointNow(1,:),posJointNow(2,:),posJointNow(3,:),'-o','Color',[1 1 1]/2);%%%冗長自由度最適化無しのIK解を描画

TipOriX=posJointNow(:,end-1)+frameJointNow(:,1,end);
TipOriY=posJointNow(:,end-1)+frameJointNow(:,2,end);
TipOriZ=posJointNow(:,end-1)+frameJointNow(:,3,end);
plot3([posJointNow(1,end-1) TipOriX(1)],[posJointNow(2,end-1) TipOriX(2)],[posJointNow(3,end-1) TipOriX(3)],'-.r');
plot3([posJointNow(1,end-1) TipOriY(1)],[posJointNow(2,end-1) TipOriY(2)],[posJointNow(3,end-1) TipOriY(3)],'-.g');
plot3([posJointNow(1,end-1) TipOriZ(1)],[posJointNow(2,end-1) TipOriZ(2)],[posJointNow(3,end-1) TipOriZ(3)],'-.b');

%%%%%%冗長自由度を最適化IK
dTheta=0.01*pi;%評価関数の勾配を求めるための微小変分
LogPhi=NaN(Loop,1);%評価関数のログ用変数
Loop=200;

for k=1:Loop
    J=ComputeJacobian(posJointNow,frameJointNow,JointConfig);%ヤコビ行列計算
    IkError(1:3,1)=(PosTarget-posJointNow(:,end) );%IK誤差を更新
    IkError(4:6,1)=rotVecTarget-rotVecNow;

    LogError(k)=sqrt(sse(IkError));
    

    if (LogError(k) < 0.005) && (k>10)&&(mean(diff(LogPhi(k-10:k-1)))<0.001)%収束判定
%         'converge'
         
        break
    elseif  LogError(k) < 0.1%スケール変数調整
        lambda=0.1;
    elseif  LogError(k) < 1
        lambda = 0.1;        
    end
    
    PhiNow= Phi(thetaNow,LinkLength,JointConfig,posJointNow,frameJointNow);%最小化される評価関数Phiは関節角をパラメータに持つスカラー関数
    LogPhi(k)=PhiNow;
    dPhidTheta=zeros(NumJoint,1);%評価関数の関節角で微分した勾配, 長さ:NumJointの縦ベクトル
    for i=1:NumJoint%評価関数の勾配を計算
        thetaPlusDtheta = thetaNow;
        thetaPlusDtheta (i) = thetaPlusDtheta (i) + dTheta; 
        dPhidTheta(i)=( Phi(thetaPlusDtheta,LinkLength,JointConfig,posJointNow,frameJointNow) - PhiNow )/dTheta;
    end
    
    if LogError(k) < 0.1%目標値とのErrorが大きくないなら冗長自由度最適化を更新
        thetaNow=thetaNow+ lambda*(pinv(J)*IkError+(eye(size( pinv(J)*J ) ) - pinv(J)*J )*dPhidTheta );
    else                %目標値とのErrorが大きいなら冗長自由度最適化を更新しない
        thetaNow=thetaNow+ lambda*(pinv(J)*IkError );
    end
    [posJointNow,frameJointNow]=FwardKinematicsArticulatedSerialArm(thetaNow,LinkLength,JointConfig);%順運動学更新
    rotVecNow=rotationMatrixToVector(frameJointNow(:,:,end)')';%回転ベクトル更新
    %CV toolboxの関数が機械工学の座標変換と転置の関係にあるため，回転行列を転置してからrotationMatrixToVectorにいれる

    
end
if k==Loop%収束しなかったとき表示
    'Not converge'
    [k LogError(k)]
end

plIkOpt=plot3(posJointNow(1,:),posJointNow(2,:),posJointNow(3,:),'-o','Color',[1 1 1]*0);%冗長自由度最適化有りのIK解を描画

TipOriX=posJointNow(:,end-1)+frameJointNow(:,1,end);
TipOriY=posJointNow(:,end-1)+frameJointNow(:,2,end);
TipOriZ=posJointNow(:,end-1)+frameJointNow(:,3,end);
plot3([posJointNow(1,end-1) TipOriX(1)],[posJointNow(2,end-1) TipOriX(2)],[posJointNow(3,end-1) TipOriX(3)],'-.r');
plot3([posJointNow(1,end-1) TipOriY(1)],[posJointNow(2,end-1) TipOriY(2)],[posJointNow(3,end-1) TipOriY(3)],'-.g');
plot3([posJointNow(1,end-1) TipOriZ(1)],[posJointNow(2,end-1) TipOriZ(2)],[posJointNow(3,end-1) TipOriZ(3)],'-.b');

%%%評価関数ログの描画用Fig 2
fig=figure(2);
fig.Units='centimeter';
fig.Position=[11 1 10 10];

plot(LogPhi)
grid on
box on
xlabel('Iteration');
ylabel('\Phi');
set(gca,'FontName','Times New Roman','FontSize',10);
ylim([-5 5])

%%%Posture描画用Fig 1の凡例
fig=figure(1);
lg=legend([plBase plTarget plIk plIkOpt],{'Base','Target','IK without Opt','IK with Opt'},'Location','best');

%%
%%
%%% my function
function [posJoint,frameJoint]=FwardKinematicsArticulatedSerialArm(theta,LengthLink,AxisOfJointConfig)
            
    NumJoint=length(theta);
         
%     directionAxis=zeros(3,NumJoint);
    
    frameJoint=zeros(3,3,NumJoint);%関節ローカル座標系を慣性系からみた[x y z]
    for i = 1:NumJoint
        frameJoint(:,:,i)=eye(3,3);
    end

    posJoint=zeros(3,NumJoint+1);%最後の列はエンドエフェクタの座標
%     vecLengthLink=zeros(3,10);%慣性系から見た　各関節に各節先端の荷重が及ぼすモーメントの腕ベクトル

    %運動学計算
    for i=1:NumJoint
       for j=1:i
           frameJoint(:,:,i)=frameJoint(:,:,i)*Raxis(theta(j),AxisOfJointConfig(:,j));
       end
       posJoint(:,i+1)=posJoint(:,i)+LengthLink(i)*frameJoint(:,1,i);
%        vecLengthLink(:,i)=LengthLink(i)*frameJoint(:,1,i);
%        directionAxis(:,i)=frameJoint(:,:,i)*AxisOfJointConfig(:,i);
    end
end


function J=ComputeJacobian(posJoint,frameJoint,JointConfig)%ヤコビ行列計算
    NumJoint=size(JointConfig,2);
    
    J=zeros(6,NumJoint);
    for i=1:NumJoint
        J(1:3,i)=cross( frameJoint(:,:,i)*JointConfig(:,i), (posJoint(:,end)-posJoint(:,i) ) );%並進成分
        J(4:6,i)=frameJoint(:,:,i)*JointConfig(:,i);%回転成分
    end

end

function R=Raxis(t,axis)%ロドリゲス回転公式 t:radian, axis:縦単位ベクトル
n=axis;
R= [cos(t)+n(1)^2*(1-cos(t)) n(1)*n(2)*(1-cos(t))-n(3)*sin(t) n(1)*n(3)*(1-cos(t))+n(2)*sin(t);
    n(2)*n(1)*(1-cos(t))+n(3)*sin(t) cos(t)+n(2)^2*(1-cos(t)) n(2)*n(3)*(1-cos(t))-n(1)*sin(t);
    n(3)*n(1)*(1-cos(t))-n(2)*sin(t) n(3)*n(2)*(1-cos(t))+n(1)*sin(t) cos(t)+n(3)^2*(1-cos(t))];
end

function OptVal = Phi(thetaP,LengthLink,JointConfig,posJoint,frameJoint)%最大化されるスカラー関数Phi
    [posJointP,frameJoinP]=FwardKinematicsArticulatedSerialArm(thetaP,LengthLink,JointConfig);
%     OptVal=-sum( (posJointP(:,4)).^2 );%4関節目の位置の二乗和を最小化
%     OptVal=-posJointP(3,4);%4関節目のz座標を最小化
    OptVal=-posJointP(2,4);%4関節目のy座標を最大化
end