#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include <iostream>

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

//int boneIndex = 0;      // root
//int startFrame = 200;   // Frame: 200 - 500
//int endFrame = 500;
//int axis = 2;           // rotation around z axis

int boneIndex = 2;      // lfemur
int startFrame = 600;   // Frame: 600 - 800
int endFrame = 800;
int axis = 0;           // rotation around x axis

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  std::cout << "# frame" << "\t" << "input" << "\t" << "linear Euler" << std::endl;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = Lerp(t, startPosture->root_pos, endPosture->root_pos);

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++){
          interpolatedPosture.bone_rotation[bone] = Lerp(t, startPosture->bone_rotation[bone], endPosture->bone_rotation[bone]);
          if ((bone == boneIndex) && (startKeyframe + frame >= startFrame) && (startKeyframe + frame <= endFrame))
          {
              if (startKeyframe == 0)
                  std::cout << (startKeyframe) << "\t" << startPosture->bone_rotation[bone][axis] << "\t" << startPosture->bone_rotation[bone][axis] << std::endl;
              Posture *inputPosture = pInputMotion->GetPosture(startKeyframe + frame);
              std::cout << (startKeyframe + frame) << "\t" << inputPosture->bone_rotation[bone][axis] << "\t" << interpolatedPosture.bone_rotation[bone][axis] << std::endl;
              if (frame == N)
                  std::cout << (startKeyframe + N + 1) << "\t" << endPosture->bone_rotation[bone][axis] << "\t" << endPosture->bone_rotation[bone][axis] << std::endl;
          }
      }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  double a = angles[0]*M_PI/180.;
  double b = angles[1]*M_PI/180.;
  double c = angles[2]*M_PI/180.;
  R[0] = cos(c) * cos(b); R[1] = cos(c) * sin(b) * sin(a) - sin(c) * cos(a); R[2] = cos(c) * sin(b) * cos(a) + sin(c) * sin(a);
  R[3] = sin(c) * cos(b); R[4] = sin(c) * sin(b) * sin(a) + cos(c) * cos(a); R[5] = sin(c) * sin(b) * cos(a) - cos(c) * sin(a);
  R[6] = -sin(b);   R[7] = cos(b) * sin(a); R[8] = cos(b) * cos(a);
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    std::cout << "# frame" << "\t" << "input" << "\t" << "Bezier Euler" << std::endl;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // create control points for Bezier
        Posture aPosture, bPosture;

        int prevKeyframe = startKeyframe - N - 1;
        int nextKeyframe = endKeyframe + N + 1;
        Posture *prevPosture, *nextPosture;
        vector intermediate_pos, intermediate_rot;

        if (startKeyframe == 0)
        {
            // p1 - startPosture, p2 - endPosture, p3 - nextPosture
            nextPosture = pInputMotion->GetPosture(nextKeyframe);

            // a1 = lerp(p1, lerp(p3, p2, 2), 1/3)
            intermediate_pos = Lerp(2.0, nextPosture->root_pos, endPosture->root_pos);
            aPosture.root_pos = Lerp(1.0/3, startPosture->root_pos, intermediate_pos);
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                intermediate_rot = Lerp(2.0, nextPosture->bone_rotation[bone], endPosture->bone_rotation[bone]);
                aPosture.bone_rotation[bone] = Lerp(1.0/3, startPosture->bone_rotation[bone], intermediate_rot);
            }

            // b2 = lerp(p2, lerp(lerp(p1, p2, 2), p3, 0.5), -1/3)
            intermediate_pos = Lerp(0.5, Lerp(2.0, startPosture->root_pos, endPosture->root_pos), nextPosture->root_pos);
            bPosture.root_pos = Lerp(-1.0/3, endPosture->root_pos, intermediate_pos);
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                intermediate_rot = Lerp(0.5, Lerp(2.0, startPosture->bone_rotation[bone], endPosture->bone_rotation[bone]), nextPosture->bone_rotation[bone]);
                bPosture.bone_rotation[bone] = Lerp(-1.0/3, endPosture->bone_rotation[bone], intermediate_rot);
            }
        } else if (endKeyframe + N + 1 >= inputLength)
        {
            // p_n-1 - prevPosture, p_n - startPosture, p_n+1 - endPosture
            prevPosture = pInputMotion->GetPosture(prevKeyframe);

            // an = lerp(p_n, lerp(lerp(p_n-1, p_n,2), p_n+1, 0.5), 1/3)
            intermediate_pos = Lerp(0.5, Lerp(2, prevPosture->root_pos, startPosture->root_pos), endPosture->root_pos);
            aPosture.root_pos = Lerp(1.0/3, startPosture->root_pos, intermediate_pos);
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                intermediate_rot = Lerp(0.5, Lerp(2, prevPosture->bone_rotation[bone], startPosture->bone_rotation[bone]), endPosture->bone_rotation[bone]);
                aPosture.bone_rotation[bone] = Lerp(1.0/3, startPosture->bone_rotation[bone], intermediate_rot);
            }

            // b_n+1 = lerp(p_n+1, lerp(p_n-1, p_n, 2), 1/3)
            intermediate_pos = Lerp(2, prevPosture->root_pos, startPosture->root_pos);
            bPosture.root_pos = Lerp(1.0/3, endPosture->root_pos, intermediate_pos);
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                intermediate_rot = Lerp(2, prevPosture->bone_rotation[bone], startPosture->bone_rotation[bone]);
                bPosture.bone_rotation[bone] = Lerp(1.0/3, endPosture->bone_rotation[bone], intermediate_rot);
            }
        } else
        {
            // p_n-1 - prevPosture, p_n - startPosture, p_n+1 - endPosture, p_n+2 - nextPosture
            prevPosture = pInputMotion->GetPosture(prevKeyframe);
            nextPosture = pInputMotion->GetPosture(nextKeyframe);

            // an = lerp(p_n, lerp(lerp(p_n-1, p_n, 2), p_n+1, 0.5), 1/3)
            intermediate_pos = Lerp(0.5, Lerp(2, prevPosture->root_pos, startPosture->root_pos), endPosture->root_pos);
            aPosture.root_pos = Lerp(1.0/3, startPosture->root_pos, intermediate_pos);
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                intermediate_rot = Lerp(0.5, Lerp(2, prevPosture->bone_rotation[bone], startPosture->bone_rotation[bone]), endPosture->bone_rotation[bone]);
                aPosture.bone_rotation[bone] = Lerp(1.0/3, startPosture->bone_rotation[bone], intermediate_rot);
            }

            // b_n+1 = lerp(p_n+1, lerp(lerp(p_n, p_n+1, 2), p_n+2, 0.5), -1/3)
            intermediate_pos = Lerp(0.5, Lerp(2, startPosture->root_pos, endPosture->root_pos), nextPosture->root_pos);
            bPosture.root_pos = Lerp(-1.0/3, endPosture->root_pos, intermediate_pos);
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                intermediate_rot = Lerp(0.5, Lerp(2, startPosture->bone_rotation[bone], endPosture->bone_rotation[bone]), nextPosture->bone_rotation[bone]);
                bPosture.bone_rotation[bone] = Lerp(-1.0/3, endPosture->bone_rotation[bone], intermediate_rot);
            }
        }

        // interpolate in between pn, an, b_n+1, p_n+1
        for(int frame=1; frame<=N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N+1);

            // interpolate root position
            interpolatedPosture.root_pos = DeCasteljauEuler(t, startPosture->root_pos, aPosture.root_pos, bPosture.root_pos, endPosture->root_pos);

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, startPosture->bone_rotation[bone], aPosture.bone_rotation[bone], bPosture.bone_rotation[bone], endPosture->bone_rotation[bone]);
                if ((bone == boneIndex) && (startKeyframe + frame >= startFrame) && (startKeyframe + frame <= endFrame))
                {
                    if (startKeyframe == 0)
                        std::cout << (startKeyframe) << "\t" << startPosture->bone_rotation[bone][axis] << "\t" << startPosture->bone_rotation[bone][axis] << std::endl;
                    Posture *inputPosture = pInputMotion->GetPosture(startKeyframe + frame);
                    std::cout << (startKeyframe + frame) << "\t" << inputPosture->bone_rotation[bone][axis] << "\t" << interpolatedPosture.bone_rotation[bone][axis] << std::endl;
                    if (frame == N)
                        std::cout << (startKeyframe + N + 1) << "\t" << endPosture->bone_rotation[bone][axis] << "\t" << endPosture->bone_rotation[bone][axis] << std::endl;
                }
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    std::cout << "# frame" << "\t" << "input" << "\t" << "SLERP quaternion" << std::endl;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for(int frame=1; frame<=N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N+1);

            // interpolate root position - using linear euler
            interpolatedPosture.root_pos = Lerp(t, startPosture->root_pos, endPosture->root_pos);

            // interpolate bone rotations - using slerp
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                // convert euler angles to quaternion
                Quaternion<double> interpolatedQuat, startQuat, endQuat;

                double startAngles[3], endAngles[3], interpolatedAngles[3];
                startPosture->bone_rotation[bone].getValue(startAngles);
                endPosture->bone_rotation[bone].getValue(endAngles);

                Euler2Quaternion(startAngles, startQuat);
                Euler2Quaternion(endAngles, endQuat);

                // interpolate
                interpolatedQuat = Slerp(t, startQuat, endQuat);

                // convert quaternion back to euler angles
                Quaternion2Euler(interpolatedQuat, interpolatedAngles);
                interpolatedPosture.bone_rotation[bone].setValue(interpolatedAngles[0], interpolatedAngles[1], interpolatedAngles[2]);

                if ((bone == boneIndex) && (startKeyframe + frame >= startFrame) && (startKeyframe + frame <= endFrame))
                {
                    if (startKeyframe == 0)
                        std::cout << (startKeyframe) << "\t" << startPosture->bone_rotation[bone][axis] << "\t" << startPosture->bone_rotation[bone][axis] << std::endl;
                    Posture *inputPosture = pInputMotion->GetPosture(startKeyframe + frame);
                    std::cout << (startKeyframe + frame) << "\t" << inputPosture->bone_rotation[bone][axis] << "\t" << interpolatedPosture.bone_rotation[bone][axis] << std::endl;
                    if (frame == N)
                        std::cout << (startKeyframe + N + 1) << "\t" << endPosture->bone_rotation[bone][axis] << "\t" << endPosture->bone_rotation[bone][axis] << std::endl;
                }
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    std::cout << "# frame" << "\t" << "input" << "\t" << "Bezier SLERP quaternion" << std::endl;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // control points for Bezier Euler
        Posture aPosture, bPosture;
        // control points for Bezier Quaternion
        Quaternion<double> aQuats[MAX_BONES_IN_ASF_FILE], bQuats[MAX_BONES_IN_ASF_FILE];

        int prevKeyframe = startKeyframe - N - 1;
        int nextKeyframe = endKeyframe + N + 1;

        Posture *prevPosture, *nextPosture;

        vector intermediate_pos;
        Quaternion<double> intermediate_rot;

        if (startKeyframe == 0)
        {
            // p1 - startPosture, p2 - endPosture, p3 - nextPosture
            nextPosture = pInputMotion->GetPosture(nextKeyframe);

            // a1 = lerp(p1, lerp(p3, p2, 2), 1/3) - for root position
            intermediate_pos = Lerp(2.0, nextPosture->root_pos, endPosture->root_pos);
            aPosture.root_pos = Lerp(1.0/3, startPosture->root_pos, intermediate_pos);
            // a1 = slerp(p1, slerp(p3, p2, 2), 1/3) - for bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                Quaternion<double> startQuat, endQuat, nextQuat;

                Euler2Quaternion(startPosture->bone_rotation[bone].p, startQuat);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, endQuat);
                Euler2Quaternion(nextPosture->bone_rotation[bone].p, nextQuat);

                intermediate_rot = Double(nextQuat, endQuat);
                aQuats[bone] = Slerp(1.0/3, startQuat, intermediate_rot);
            }

            // b2 = lerp(p2, lerp(lerp(p1, p2, 2), p3, 0.5), -1/3) - for root position
            intermediate_pos = Lerp(0.5, Lerp(2.0, startPosture->root_pos, endPosture->root_pos), nextPosture->root_pos);
            bPosture.root_pos = Lerp(-1.0/3, endPosture->root_pos, intermediate_pos);
            // b2 = slerp(p2, slerp(slerp(p1, p2, 2), p3, 0.5), -1/3) - for bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                Quaternion<double> startQuat, endQuat, nextQuat;

                Euler2Quaternion(startPosture->bone_rotation[bone].p, startQuat);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, endQuat);
                Euler2Quaternion(nextPosture->bone_rotation[bone].p, nextQuat);

                intermediate_rot = Double( startQuat, endQuat);
                intermediate_rot = Slerp(0.5, intermediate_rot, nextQuat);
                bQuats[bone] = Slerp(-1.0/3, endQuat, intermediate_rot);
            }
        } else if (endKeyframe + N + 1 >= inputLength)
        {
            // p_n-1 - prevPosture, p_n - startPosture, p_n+1 - endPosture
            prevPosture = pInputMotion->GetPosture(prevKeyframe);

            // an = lerp(p_n, lerp(lerp(p_n-1, p_n,2), p_n+1, 0.5), 1/3) - for root position
            intermediate_pos = Lerp(0.5, Lerp(2, prevPosture->root_pos, startPosture->root_pos), endPosture->root_pos);
            aPosture.root_pos = Lerp(1.0/3, startPosture->root_pos, intermediate_pos);
            // an = slerp(p_n, slerp(slerp(p_n-1, p_n,2), p_n+1, 0.5), 1/3) - for bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                Quaternion<double> prevQuat, startQuat, endQuat;

                Euler2Quaternion(prevPosture->bone_rotation[bone].p, prevQuat);
                Euler2Quaternion(startPosture->bone_rotation[bone].p, startQuat);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, endQuat);

                intermediate_rot = Double(prevQuat, startQuat);
                intermediate_rot = Slerp(0.5, intermediate_rot, endQuat);
                aQuats[bone] = Slerp(1.0/3, startQuat, intermediate_rot);
            }

            // b_n+1 = lerp(p_n+1, lerp(p_n-1, p_n, 2), 1/3) - for root position
            intermediate_pos = Lerp(2, prevPosture->root_pos, startPosture->root_pos);
            bPosture.root_pos = Lerp(1.0/3, endPosture->root_pos, intermediate_pos);

            // b_n+1 = slerp(p_n+1, slerp(p_n-1, p_n, 2), 1/3) - for bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                Quaternion<double> prevQuat, startQuat, endQuat;

                Euler2Quaternion(prevPosture->bone_rotation[bone].p, prevQuat);
                Euler2Quaternion(startPosture->bone_rotation[bone].p, startQuat);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, endQuat);

                intermediate_rot = Double(prevQuat, startQuat);
                bQuats[bone] = Slerp(1.0/3, endQuat, intermediate_rot);
            }
        } else
        {
            // p_n-1 - prevPosture, p_n - startPosture, p_n+1 - endPosture, p_n+2 - nextPosture
            prevPosture = pInputMotion->GetPosture(prevKeyframe);
            nextPosture = pInputMotion->GetPosture(nextKeyframe);

            // an = lerp(p_n, lerp(lerp(p_n-1, p_n, 2), p_n+1, 0.5), 1/3) - for root position
            intermediate_pos = Lerp(0.5, Lerp(2, prevPosture->root_pos, startPosture->root_pos), endPosture->root_pos);
            aPosture.root_pos = Lerp(1.0/3, startPosture->root_pos, intermediate_pos);

            // an = slerp(p_n, slerp(slerp(p_n-1, p_n, 2), p_n+1, 0.5), 1/3) - for bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                Quaternion<double> prevQuat, startQuat, endQuat;

                Euler2Quaternion(prevPosture->bone_rotation[bone].p, prevQuat);
                Euler2Quaternion(startPosture->bone_rotation[bone].p, startQuat);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, endQuat);

                intermediate_rot = Double(prevQuat, startQuat);
                intermediate_rot = Slerp(0.5, intermediate_rot, endQuat);
                aQuats[bone] = Slerp(1.0/3, startQuat, intermediate_rot);
            }

            // b_n+1 = lerp(p_n+1, lerp(lerp(p_n, p_n+1, 2), p_n+2, 0.5), -1/3) - for root position
            intermediate_pos = Lerp(0.5, Lerp(2, startPosture->root_pos, endPosture->root_pos), nextPosture->root_pos);
            bPosture.root_pos = Lerp(-1.0/3, endPosture->root_pos, intermediate_pos);

            // b_n+1 = slerp(p_n+1, slerp(slerp(p_n, p_n+1, 2), p_n+2, 0.5), -1/3) - for root position - for bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                Quaternion<double> startQuat, endQuat, nextQuat;

                Euler2Quaternion(startPosture->bone_rotation[bone].p, startQuat);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, endQuat);
                Euler2Quaternion(nextPosture->bone_rotation[bone].p, nextQuat);

                intermediate_rot = Double(startQuat, endQuat);
                intermediate_rot = Slerp(0.5, intermediate_rot, nextQuat);
                bQuats[bone] = Slerp(1.0/3, endQuat, intermediate_rot);
            }
        }

        // interpolate in between pn, an, b_n+1, p_n+1
        for(int frame=1; frame<=N; frame++)
        {
            Posture interpolatedPosture;
            Quaternion<double> interpolatedQuat;
            double t = 1.0 * frame / (N+1);

            // interpolate root position - using bezier euler
            interpolatedPosture.root_pos = DeCasteljauEuler(t, startPosture->root_pos, aPosture.root_pos, bPosture.root_pos, endPosture->root_pos);

            // interpolate bone rotations - using bezier quaternion
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                // convert start and end euler angles to quaternions
                Quaternion<double> startQuat, endQuat;

                Euler2Quaternion(startPosture->bone_rotation[bone].p, startQuat);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, endQuat);

                // Decasteljau construction
                interpolatedQuat = DeCasteljauQuaternion(t, startQuat, aQuats[bone], bQuats[bone], endQuat);

                // convert quaternion back to euler angle and store it into interpolated posture
                double interpolatedAngles[3];
                Quaternion2Euler(interpolatedQuat, interpolatedAngles);
                interpolatedPosture.bone_rotation[bone].setValue(interpolatedAngles[0], interpolatedAngles[1], interpolatedAngles[2]);

                if ((bone == boneIndex) && (startKeyframe + frame >= startFrame) && (startKeyframe + frame <= endFrame))
                {
                    if (startKeyframe == 0)
                        std::cout << (startKeyframe) << "\t" << startPosture->bone_rotation[bone][axis] << "\t" << startPosture->bone_rotation[bone][axis] << std::endl;
                    Posture *inputPosture = pInputMotion->GetPosture(startKeyframe + frame);
                    std::cout << (startKeyframe + frame) << "\t" << inputPosture->bone_rotation[bone][axis] << "\t" << interpolatedPosture.bone_rotation[bone][axis] << std::endl;
                    if (frame == N)
                        std::cout << (startKeyframe + N + 1) << "\t" << endPosture->bone_rotation[bone][axis] << "\t" << endPosture->bone_rotation[bone][axis] << std::endl;
                }
            }
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  // first convert euler angles to rotation matrix
  double R[9];
  Euler2Rotation(angles, R);
  // then covert rotation matrix to quaternion
  q = q.Matrix2Quaternion(R);
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  // first convert quaternion to rotation matrix
  double R[9];
  q.Quaternion2Matrix(R);
  // then covert rotation matrix to euler angles
  Rotation2Euler(R, angles);
}

vector Interpolator::Lerp(double t, vector vStart, vector vEnd) {
    return (vStart * (1 - t) + vEnd * t);
}

Quaternion<double> Interpolator::Lerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd)
{
    return (qStart * (1 - t) + qEnd * t);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd)
{
    qStart.Normalize();
    qEnd.Normalize();

    // find the shortest path
    double dot = qStart.dot(qEnd);
    if (dot < 0)
    {
        qStart = qStart * (-1);
        dot = -dot;
    }

    double theta = acos(dot);

    Quaternion<double> result;
    result = sin((1-t)*theta)/sin(theta) * qStart + sin(t*theta)/sin(theta) * qEnd;

    if (isnan(result.Gets()))
        result = Lerp(t, qStart, qEnd);

    return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  return Slerp(2.0, p, q);
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  vector result;
  vector q0, q1, q2, r0, r1;
  q0 = Lerp(t, p0, p1);
  q1 = Lerp(t, p1, p2);
  q2 = Lerp(t, p2, p3);
  r0 = Lerp(t, q0, q1);
  r1 = Lerp(t, q1, q2);
  result = Lerp(t, r0, r1);
  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  Quaternion<double> result;
  Quaternion<double> q0, q1, q2, r0, r1;
  q0 = Slerp(t, p0, p1);
  q1 = Slerp(t, p1, p2);
  q2 = Slerp(t, p2, p3);
  r0 = Slerp(t, q0, q1);
  r1 = Slerp(t, q1, q2);
  result = Slerp(t, r0, r1);
  return result;
}

