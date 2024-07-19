import tensorflow as tf
from inversekinematics_rc_real import *
from math import *
import numpy as np

WaypointModel = tf.keras.models.load_model('Waypoint_RC_R9_230917.h5')

l1=6.24565
l2=3.11364
l3=1.910051

class excavation_path():
    def lineBasicInfo(self, lineVel, LineEndDist, LineEndDept, LineEndAOA):
        self.MaxVelLine = lineVel  # 3
        self.LineEndDist = LineEndDist
        self.LineEndHeight = LineEndDept
        self.LineEndAOA = LineEndAOA

    def linepath(self, curSwing, curDist, curDept, curAOA):
        TrajectoryDataReal, JointVariables, time = [], [], []

        Initiation_Distance = np.sqrt(np.power((curDist - self.LineEndDist), 2) + np.power((curDept - self.LineEndHeight), 2))
        timeofStep = 3 * Initiation_Distance / (2 * self.MaxVelLine)
        numofStep = int(timeofStep/0.1)
        self.dirTheta = atan2(self.LineEndHeight - curDept, self.LineEndDist - curDist) * 180 / pi

        initaoa = curAOA

        for a in range(0, numofStep+1):
            if a/10 < timeofStep/3:
                vel = 3 * self.MaxVelLine * (a/10) / timeofStep
            elif a/10 >= timeofStep/3 and a/10 < 2*timeofStep/3:
                vel = self.MaxVelLine
            else:
                vel = - (3 * self.MaxVelLine) * (a/10 - timeofStep) / timeofStep

            curDist += vel * cos(radians(self.dirTheta)) * 0.1
            curDept += vel * sin(radians(self.dirTheta)) * 0.1
            curAOA = initaoa + (self.LineEndAOA - initaoa) * (a/10) / timeofStep  # radians

            boomAng, armAng, bktAng = inversekinematics_real(curDist, curDept, curAOA)
            boomAng, armAng, bktAng = degrees(boomAng), degrees(armAng), degrees(bktAng)

            if boomAng > 59.6:
                boomAng = 59.6
            elif boomAng < 0:
                boomAng = 0

            if armAng > -34.25:
                armAng = -34.25
            elif armAng < -159.1:
                armAng = -159.1

            if bktAng > 37:
                bktAng = 37
            elif bktAng < -133.6:
                bktAng = -133.6

            curDist_new, curDept_new, curAOA_new = forwardkinematics_real(radians(boomAng),radians(armAng),radians(bktAng))
            TrajectoryDataReal.append([curDist_new, curDept_new, curAOA_new])
            JointVariables.append([curSwing, boomAng, armAng, bktAng])

            time.append(a/10)
        return JointVariables, TrajectoryDataReal, time

    def basicinput(self, drag_length, terrain_avg_depth, SoilHeightAdjustment, depth_limit, initDist, init_pose, LiftVel):
        self.ExcavationLength = drag_length / 10
        self.SoilMeanDepth = terrain_avg_depth / 10
        self.SoilDepthAdjustment = SoilHeightAdjustment/10
        self.aiExcInitDist = initDist / 10
        self.depthLimit = depth_limit

        self.LiftEndDist = init_pose[0]
        self.LiftEndHeight = init_pose[1]
        self.LiftAOA = init_pose[2]
        self.MaxVelLift = LiftVel

    def pathgeneration(self, curSwing):# AI 관련
        # TrajectoryDataReal, JointVariables, time = [], [], []

        # x_predict = np.array([[self.ExcavationLength, self.SoilMeanDepth]])
        # x_predict = x_predict.reshape(x_predict.shape[0], x_predict.shape[1], 1)
        # y_predict = WaypointModel.predict(x_predict)

        # traj_coeff_2nd = y_predict[0][0] * 5
        # traj_coeff_1st = y_predict[0][1] * 5
        # traj_coeff_0th = y_predict[0][2] * 2

        # x_predict, y_predict = [], []
        # for a in range(0, 71):
        #     bkt_dist = -a * self.ExcavationLength / 70 + self.aiExcInitDist
        #     diff = traj_coeff_2nd * (0.9 ** 2) + traj_coeff_1st * 0.9 + traj_coeff_0th - self.SoilDepthAdjustment
        #     bkt_dept = traj_coeff_2nd * ((bkt_dist-self.aiExcInitDist+0.9) ** 2) + traj_coeff_1st * (bkt_dist-self.aiExcInitDist+0.9) + traj_coeff_0th - diff
        #     bkt_dist = bkt_dist * 10
        #     bkt_dept = bkt_dept * 10
        #     if a == 0:
        #         lambda_theta = np.clip((bkt_dist ** 2 + bkt_dept ** 2 + (l2 + l3) ** 2 - l1 ** 2) / (
        #                 2 * sqrt(bkt_dist ** 2 + bkt_dept ** 2) * (l2 + l3)), -1, 1)
        #         bkt_aoa = atan2(bkt_dept, bkt_dist) - acos(lambda_theta) + radians(10)
        #         initAoa = bkt_aoa
        #     else:
        #         bkt_aoa = a * (radians(-137) - initAoa) / 70 + initAoa

        #     if bkt_dept <= self.depthLimit:
        #         bkt_dept = self.depthLimit

        #     boomang, armang, bktang = inversekinematics_real(bkt_dist, bkt_dept, bkt_aoa)
        #     boomang, armang, bktang = degrees(boomang), degrees(armang), degrees(bktang)
        #     Dist_trajectory, Dept_trajectory, AOA_trajectory = forwardkinematics_real(radians(boomang), radians(armang), radians(bktang))

        #     TrajectoryDataReal.append([Dist_trajectory, Dept_trajectory, AOA_trajectory])
        #     JointVariables.append([curSwing, boomang, armang, bktang])
        #     time.append(a / 10)

        # self.lastBoomAng, self.lastArmAng, self.lastBktAng = JointVariables[-1][1], JointVariables[-1][2], JointVariables[-1][3]

        # curDist, curDept, curAOA = forwardkinematics_real(radians(self.lastBoomAng), radians(self.lastArmAng), radians(self.lastBktAng))
        # initaoa = curAOA

        # LiftDistance = np.sqrt(np.power((curDist - self.LiftEndDist), 2) + np.power((curDept - self.LiftEndHeight), 2))
        # LiftTime = 12 * LiftDistance / (7 * self.MaxVelLift)

        # dirTheta = atan2(self.LiftEndHeight - curDept, self.LiftEndDist - curDist) * 180 / pi

        # numLift = int(round(LiftTime, 1) / 0.1)
        # for c in range(1, numLift + 1):
        #     if c / 10 < 2 * LiftTime / 6:
        #         vel = 3 * self.MaxVelLift * (c / 10) / LiftTime
        #     elif c / 10 >= 2 * LiftTime / 6 and c / 10 < 3 * LiftTime / 6:
        #         vel = self.MaxVelLift
        #     else:
        #         vel = self.MaxVelLift - (2 * self.MaxVelLift) * ((c / 10) - 3 * LiftTime / 6) / LiftTime

        #     curDist += vel * cos(radians(dirTheta)) * 0.1
        #     curDept += vel * sin(radians(dirTheta)) * 0.1
        #     curAOA = initaoa + (self.LiftAOA - initaoa) * (c / 10) / LiftTime
        #     TrajectoryDataReal.append([curDist, curDept, curAOA])

        #     BoomAng1, ArmAng1, BktAng1 = inversekinematics_real(curDist, curDept, curAOA)
        #     self.lastBoomAng, self.lastArmAng, self.lastBktAng = degrees(BoomAng1), degrees(ArmAng1), degrees(BktAng1)

        #     if self.lastBoomAng > 59.6:
        #         self.lastBoomAng = 59.6
        #     elif self.lastBoomAng < 0:
        #         self.lastBoomAng = 0

        #     if self.lastArmAng > -34.25:
        #         self.lastArmAng = -34.25
        #     elif self.lastArmAng < -159.1:
        #         self.lastArmAng = -159.1

        #     if self.lastBktAng > 37:
        #         self.lastBktAng = 37
        #     elif self.lastBktAng < -133.6:
        #         self.lastBktAng = -133.6

        #     curDist_new, curDept_new, curAOA_new = forwardkinematics_real(radians(self.lastBoomAng), radians(self.lastArmAng), radians(self.lastBktAng))
        #     TrajectoryDataReal.append([curDist_new, curDept_new, curAOA_new])
        #     JointVariables.append([curSwing, self.lastBoomAng, self.lastArmAng, self.lastBktAng])

        #     liftingTime = time[-1] + 0.1
        #     time.append(liftingTime)

        # return JointVariables, TrajectoryDataReal, time
        return 0

    def swingInfo(self, swingvel, swingGoal):
        self.swingVelocity = swingvel
        self.swingGoalAng = swingGoal

    def swingPath(self, curSwing, curDist, curDept, curAOA):
        TrajectoryDataReal, JointVariables, time = [], [], []

        if self.swingGoalAng > 180 and curSwing < 0:
            swingTimeStep = 1.5 * (-360 + self.swingGoalAng - curSwing) / self.swingVelocity
        elif self.swingGoalAng < -180 and curSwing > 0:
            swingTimeStep = 1.5 * (360 + self.swingGoalAng - curSwing) / self.swingVelocity
        else:
            swingTimeStep = 1.5 * (self.swingGoalAng - curSwing) / self.swingVelocity

        boomAng, armAng, bktAng = inversekinematics_real(curDist, curDept, curAOA)
        boomAng, armAng, bktAng = degrees(boomAng), degrees(armAng), degrees(bktAng)

        if swingTimeStep < 0:
            self.swingVelocity = -self.swingVelocity
            swingTimeStep = -swingTimeStep

        numSwing = int(swingTimeStep/0.1)
        for a in range(0, numSwing+1):
            if a/10 < swingTimeStep / 3:
                vel = 3 * self.swingVelocity * (a/10) / swingTimeStep
            elif a/10 >= swingTimeStep / 3 and a/10 < 2 * swingTimeStep / 3:
                vel = self.swingVelocity
            else:
                vel = (-3 * self.swingVelocity * (a/10) / swingTimeStep) + 3 * self.swingVelocity

            curSwing += vel * 0.1

            if curSwing > 180:
                curSwing = -360 + curSwing
            elif curSwing < -180:
                curSwing = 360 + curSwing

            TrajectoryDataReal.append([curDist, curDept, curAOA])
            JointVariables.append([curSwing, boomAng, armAng, bktAng])
            time.append(a / 10)

        return JointVariables, TrajectoryDataReal, time

    def loading_Track_info(self, loading_init_dist, loading_init_depth, loading_final_dist, loading_radius, loading_vel, loading_aoa):
        self.loadingInit = loading_init_dist
        self.loadingHeight = loading_init_depth
        self.loadingRadius = loading_radius
        self.loadingFinal = loading_final_dist
        self.loadVel = loading_vel
        self.loadFinalAOA = loading_aoa

    def loading_Track(self, curSwing, curDist, curDept, curAOA):
        TrajectoryDataReal, JointVariables, time = [], [], []
        dist_range = self.loadingFinal - self.loadingInit
        drag_range = dist_range - self.loadingRadius

        path_length = drag_range + self.loadingRadius * radians(86)

        time_loading = 3 * path_length / (2 * self.loadVel)
        step_loading = int(time_loading / 0.1)

        curMovingDistance = 0
        for a in range(0, step_loading+1):
            if a/10 < time_loading/3:
                vel = 3 * self.loadVel * (a/10) / time_loading
            elif a/10 >= time_loading/3 and a/10 < 2*time_loading/3:
                vel = self.loadVel
            else:
                vel = - (3 * self.loadVel) * (a/10 - time_loading) / time_loading

            curMovingDistance += vel * 0.1
            hor_pos = curMovingDistance
            ver_pos = 0

            if curMovingDistance >= drag_range:
                hor_pos = drag_range + self.loadingRadius * sin((curMovingDistance - drag_range)/self.loadingRadius)
                ver_pos = self.loadingRadius - (self.loadingRadius * cos((curMovingDistance - drag_range)/self.loadingRadius))

            curDist_new = curDist + hor_pos
            curDept_new = curDept + ver_pos
            curAOA_new = curAOA + (self.loadFinalAOA - curAOA) * (a / 10) / time_loading  # radians

            boomAng, armAng, bktAng = inversekinematics_real(curDist_new, curDept_new, curAOA_new)
            boomAng, armAng, bktAng = degrees(boomAng), degrees(armAng), degrees(bktAng)

            if boomAng > 59.6:
                boomAng = 59.6
            elif boomAng < 0:
                boomAng = 0

            if armAng > -34.25:
                armAng = -34.25
            elif armAng < -159.1:
                armAng = -159.1

            if bktAng > 37:
                bktAng = 37
            elif bktAng < -133.6:
                bktAng = -133.6

            curDist_new, curDept_new, curAOA_new = forwardkinematics_real(radians(boomAng), radians(armAng), radians(bktAng))
            TrajectoryDataReal.append([curDist_new, curDept_new, curAOA_new])
            JointVariables.append([curSwing, boomAng, armAng, bktAng])
            time.append(a / 10)
        return JointVariables, TrajectoryDataReal, time

    def SoilMountainInfo(self, MountainHeight, depth_limit, initDist, finalAOA, init_pose, LiftVel):
        self.SoilMountainHeihgt = MountainHeight / 10
        self.SoilMountainExcInitDist = initDist / 10
        self.depthLimit = depth_limit
        self.excFinalAOA = finalAOA

        self.LiftEndDist = init_pose[0]
        self.LiftEndHeight = init_pose[1]
        self.LiftAOA = init_pose[2]
        self.MaxVelLift = LiftVel

    def SoilMountainExcavation(self, curSwing): # AI 관련
        # TrajectoryDataReal, JointVariables, time = [], [], []
        # TipDepth = []

        # if self.SoilMountainHeihgt < -1:
        #     x_predict = np.array([[0.42, -0.24]])  # DragLength/10, SoilMeanDepth/10
        # else:
        #     x_predict = np.array([[0.42, -0.30]])

        # x_predict = x_predict.reshape(x_predict.shape[0], x_predict.shape[1], 1)
        # y_predict = WaypointModel.predict(x_predict)

        # traj_coeff_2nd = y_predict[0][0] * 5
        # traj_coeff_1st = y_predict[0][1] * 5
        # traj_coeff_0th = y_predict[0][2] * 2
        # #x_predict, y_predict = [], []
        # for a in range(0, 101):
        #     bkt_dist = -a * 0.42 / 100 + self.SoilMountainExcInitDist
        #     diff = traj_coeff_2nd * (0.9 ** 2) + traj_coeff_1st * 0.9 + traj_coeff_0th - self.SoilMountainHeihgt
        #     bkt_dept = traj_coeff_2nd * ((bkt_dist-self.SoilMountainExcInitDist+0.9) ** 2) + traj_coeff_1st * (bkt_dist-self.SoilMountainExcInitDist+0.9) + traj_coeff_0th - diff
        #     bkt_dist = bkt_dist * 10
        #     bkt_dept = bkt_dept * 10
        #     if a == 0:
        #         lambda_theta = np.clip((bkt_dist ** 2 + bkt_dept ** 2 + (l2 + l3) ** 2 - l1 ** 2) / (2 * sqrt(bkt_dist ** 2 + bkt_dept ** 2) * (l2 + l3)), -1, 1)
        #         bkt_aoa = atan2(bkt_dept, bkt_dist) - acos(lambda_theta)
        #         initAoa = bkt_aoa
        #     else:
        #         bkt_aoa = a * (self.excFinalAOA - initAoa) / 100 + initAoa

        #     if bkt_dept <= self.depthLimit:
        #         bkt_dept = self.depthLimit

        #     boomang, armang, bktang = inversekinematics_real(bkt_dist, bkt_dept, bkt_aoa)
        #     boomang, armang, bktang = degrees(boomang), degrees(armang), degrees(bktang)

        #     if boomang > 59.6:
        #         boomang = 59.6
        #     elif boomang < 0:
        #         boomang = 0

        #     if armang > -34.25:
        #         armang = -34.25
        #     elif armang < -159.1:
        #         armang = -159.1

        #     if bktang > 37:
        #         bktang = 37
        #     elif bktang < -133.6:
        #         bktang = -133.6

        #     Dist_trajectory, Dept_trajectory, AOA_trajectory = forwardkinematics_real(radians(boomang), radians(armang), radians(bktang))
        #     TrajectoryDataReal.append([Dist_trajectory, Dept_trajectory, AOA_trajectory])
        #     TipDepth.append(Dept_trajectory)
        #     JointVariables.append([curSwing, boomang, armang, bktang])
        #     time.append(a / 10)

        # lastBoomAng, lastArmAng, lastBktAng = JointVariables[-1][1], JointVariables[-1][2], JointVariables[-1][3]

        # curDist, curDept, curAOA = forwardkinematics_real(radians(lastBoomAng), radians(lastArmAng), radians(lastBktAng))
        # initaoa = curAOA

        # LiftDistance = np.sqrt(np.power((curDist - self.LiftEndDist), 2) + np.power((curDept - self.LiftEndHeight), 2))
        # LiftTime = 12 * LiftDistance / (7 * self.MaxVelLift)

        # dirTheta = atan2(self.LiftEndHeight - curDept, self.LiftEndDist - curDist) * 180 / pi

        # numLift = int(round(LiftTime, 1) / 0.1)
        # for c in range(1, numLift + 1):
        #     if c / 10 < 2 * LiftTime / 6:
        #         vel = 3 * self.MaxVelLift * (c / 10) / LiftTime
        #     elif c / 10 >= 2 * LiftTime / 6 and c / 10 < 3 * LiftTime / 6:
        #         vel = self.MaxVelLift
        #     else:
        #         vel = self.MaxVelLift - (2 * self.MaxVelLift) * ((c / 10) - 3 * LiftTime / 6) / LiftTime

        #     curDist += vel * cos(radians(dirTheta)) * 0.1
        #     curDept += vel * sin(radians(dirTheta)) * 0.1
        #     curAOA = initaoa + (self.LiftAOA - initaoa) * (c / 10) / LiftTime
        #     TrajectoryDataReal.append([curDist, curDept, curAOA])

        #     BoomAng1, ArmAng1, BktAng1 = inversekinematics_real(curDist, curDept, curAOA)
        #     lastBoomAng, lastArmAng, lastBktAng = degrees(BoomAng1), degrees(ArmAng1), degrees(BktAng1)
        #     JointVariables.append([curSwing, lastBoomAng, lastArmAng, lastBktAng])

        #     liftingTime = time[-1] + 0.1
        #     time.append(liftingTime)

        # return JointVariables, TrajectoryDataReal, TipDepth, time
        return 0

    def final_path(self, curSwing, curDist, curDept, curAOA, FinalDist, GoalDept, Lift_dist, Lift_dept, Lift_aoa, Lift_vel):

        self.lineBasicInfo(0.8, FinalDist, GoalDept, radians(-135))
        JointVariables, TrajectoryDataReal, time = self.linepath(curSwing, curDist, curDept, curAOA)

        self.lineBasicInfo(Lift_vel, Lift_dist, Lift_dept, Lift_aoa)
        JointVariables2, TrajectoryDataReal2, time2 = self.linepath(curSwing, TrajectoryDataReal[-1][0], TrajectoryDataReal[-1][1], TrajectoryDataReal[-1][2])

        JointVariables.extend(JointVariables2)
        TrajectoryDataReal.extend(TrajectoryDataReal2)
        time.extend(time2)

        return JointVariables, TrajectoryDataReal, time





