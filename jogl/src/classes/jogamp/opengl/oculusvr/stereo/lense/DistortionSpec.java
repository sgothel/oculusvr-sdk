/**
 * Copyright 2014 JogAmp Community. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are
 * permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright notice, this list of
 *       conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright notice, this list
 *       of conditions and the following disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *
 *    3. Compliance with Oculus VR RIFT SDK LICENSE (see below)
 *
 * THIS SOFTWARE IS PROVIDED BY JogAmp Community ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL JogAmp Community OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those of the
 * authors and should not be interpreted as representing official policies, either expressed
 * or implied, of JogAmp Community.
 *
 * XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
 *
 * This file contains mathematical equations, comments and algorithms
 * used in the Oculus VR RIFT SDK 0.3.2.
 *
 * Due to unknown legal status, the 'requested' Copyright tag and disclaimer
 * below has been added.
 *
 * XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
 *
 * Copyright © 2014 Oculus VR, Inc. All rights reserved
 *
 * Oculus VR, Inc. Software Development Kit License Agreement
 *
 * Human-Readable Summary*:
 *
 * You are Free to:
 *
 * Use, modify, and distribute the Oculus VR Rift SDK in source and binary
 * form with your applications/software.
 *
 * With the Following Restrictions:
 *
 * You can only distribute or re-distribute the source code to LibOVR in
 * whole, not in part.
 *
 * Modifications to the Oculus VR Rift SDK in source or binary form must
 * be shared with Oculus VR.
 *
 * If your applications cause health and safety issues, you may lose your
 * right to use the Oculus VR Rift SDK, including LibOVR.
 *
 * The Oculus VR Rift SDK may not be used to interface with unapproved commercial
 * virtual reality mobile or non-mobile products or hardware.

 * * - This human-readable Summary is not a license. It is simply a convenient
 * reference for understanding the full Oculus VR Rift SDK License Agreement.
 * The Summary is written as a user-friendly interface to the full Oculus VR Rift
 * SDK License below. This Summary itself has no legal value, and its contents do
 * not appear in the actual license.
 *
 * Full-length Legal Copy may be found at:
 *   http://www.oculusvr.com/licenses/LICENSE-3.1
 *   http://jogamp.org/git/?p=oculusvr-sdk.git;a=blob;f=LICENSE.txt;hb=HEAD
 *   Or within this repository: oculusvr-sdk/LICENSE.txt
 *
 * THIS RIFT SDK AND ANY COMPONENT THEREOF IS PROVIDED BY OCULUS VR AND
 * ITS CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL OCULUS VR AS THE
 * COPYRIGHT OWNER OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS RIFT
 * SDK OR THE RIFT SDK DERIVATIVES, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jogamp.opengl.oculusvr.stereo.lense;

import com.jogamp.math.VectorUtil;
import com.jogamp.nativewindow.util.DimensionImmutable;
import com.jogamp.opengl.util.stereo.generic.GenericStereoDeviceConfig;

public class DistortionSpec {
    public DistortionSpec(final LensConfig lens) {
        this.lens = lens;
        this.pixelsPerTanAngleAtCenter = new float[2];
        this.tanEyeAngleScale = new float[2];
        this.lensCenter = new float[2];
    }

    private static final float[] quaterHalf = new float[] { 0.25f, 0.5f };

    final LensConfig lens;

    final float[] pixelsPerTanAngleAtCenter;
    final float[] tanEyeAngleScale;
    final float[] lensCenter;

    public static DistortionSpec[] CalculateDistortionSpec (final GenericStereoDeviceConfig deviceConfig, final float[] eyeReliefInMeters) {
        final DistortionSpec[] result = new DistortionSpec[2];
        /// FIXME: Add 'pluggable' lense configuration
        final LensConfig[] lensConfig = LensConfig.GenerateLensConfigFromEyeRelief(eyeReliefInMeters, LensConfig.DistortionEquation.CatmullRom10);
        result[0] = CalculateDistortionSpec (deviceConfig, 0, eyeReliefInMeters[0], lensConfig[0]);
        result[1] = CalculateDistortionSpec (deviceConfig, 1, eyeReliefInMeters[1], lensConfig[1]);
        return result;
    }


    private static DistortionSpec CalculateDistortionSpec (final GenericStereoDeviceConfig deviceConfig, final int eyeName,
                                                           final float eyeReliefInMeters, final LensConfig lensConfig) {
        // From eye relief, IPD and device characteristics, we get the distortion mapping.
        // This distortion does the following things:
        // 1. It undoes the distortion that happens at the edges of the lens.
        // 2. It maps the undistorted field into "retina" space.
        // So the input is a pixel coordinate - the physical pixel on the display itself.
        // The output is the real-world direction of the ray from this pixel as it comes out of the lens and hits the eye.
        // However we typically think of rays "coming from" the eye, so the direction (TanAngleX,TanAngleY,1) is the direction
        //      that the pixel appears to be in real-world space, where AngleX and AngleY are relative to the straight-ahead vector.
        // If your renderer is a raytracer, you can use this vector directly (normalize as appropriate).
        // However in standard rasterisers, we have rendered a 2D image and are putting it in front of the eye,
        //      so we then need a mapping from this space to the [-1,1] UV coordinate space, which depends on exactly
        //      where "in space" the app wants to put that rendertarget.
        //      Where in space, and how large this rendertarget is, is completely up to the app and/or user,
        //      though of course we can provide some useful hints.

        // TODO: Use IPD and eye relief to modify distortion (i.e. non-radial component)
        // TODO: cope with lenses that don't produce collimated light.
        //       This means that IPD relative to the lens separation changes the light vergence,
        //       and so we actually need to change where the image is displayed.
        final DistortionSpec localDistortion = new DistortionSpec(lensConfig);

        final DimensionImmutable resolutionInPixels = deviceConfig.surfaceSizeInPixels;

        final float[] pixelsPerMeter = new float[] { resolutionInPixels.getWidth() / deviceConfig.screenSizeInMeters[0],
                                                     resolutionInPixels.getHeight() / deviceConfig.screenSizeInMeters[1] };

        VectorUtil.scaleVec2(localDistortion.pixelsPerTanAngleAtCenter, pixelsPerMeter, localDistortion.lens.MetersPerTanAngleAtCenter);

        // Same thing, scaled to [-1,1] for each eye, rather than pixels.

        VectorUtil.scaleVec2(localDistortion.tanEyeAngleScale, quaterHalf,
                VectorUtil.divVec2(new float[2], deviceConfig.screenSizeInMeters, localDistortion.lens.MetersPerTanAngleAtCenter) );


        // <--------------left eye------------------><-ScreenGapSizeInMeters-><--------------right eye----------------->
        // <------------------------------------------ScreenSizeInMeters.Width----------------------------------------->
        //                            <------------interpupillaryDistanceInMeters-------------->
        // <--centerFromLeftInMeters->
        //                            ^
        //                      Center of lens

        // Find the lens centers in scale of [-1,+1] (NDC) in left eye.
        final float visibleWidthOfOneEye = 0.5f * ( deviceConfig.screenSizeInMeters[0] );
        final float centerFromLeftInMeters = ( deviceConfig.screenSizeInMeters[0] - deviceConfig.interpupillaryDistanceInMeters ) * 0.5f;
        localDistortion.lensCenter[0] = (     centerFromLeftInMeters / visibleWidthOfOneEye          ) * 2.0f - 1.0f;
        localDistortion.lensCenter[1] = ( deviceConfig.pupilCenterFromScreenTopInMeters  / deviceConfig.screenSizeInMeters[1] ) * 2.0f - 1.0f;
        if ( 1 == eyeName ) {
            localDistortion.lensCenter[0] = -localDistortion.lensCenter[0];
        }

        return localDistortion;
    }


}
