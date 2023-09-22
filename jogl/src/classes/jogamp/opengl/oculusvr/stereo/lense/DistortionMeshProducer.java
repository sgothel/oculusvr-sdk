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

import jogamp.opengl.util.stereo.DistortionMesh;
import jogamp.opengl.util.stereo.ScaleAndOffset2D;
import jogamp.opengl.util.stereo.DistortionMesh.DistortionVertex;

import com.jogamp.math.FloatUtil;
import com.jogamp.math.VectorUtil;
import com.jogamp.opengl.util.stereo.EyeParameter;
import com.jogamp.opengl.util.stereo.generic.GenericStereoDeviceConfig;

public class DistortionMeshProducer implements DistortionMesh.Producer {
    private DistortionSpec[] distortionSpecs;
    private GenericStereoDeviceConfig deviceConfig;
    private final float[] eyeReliefInMeters;

    public DistortionMeshProducer() {
        this.distortionSpecs = null;
        this.deviceConfig = null;
        this.eyeReliefInMeters = new float[] { 0, 0 };
    }

    @Override
    public void init(final GenericStereoDeviceConfig deviceConfig, final float[] eyeReliefInMeters) throws IllegalStateException {
        if( null != this.deviceConfig ) {
            throw new IllegalStateException("Already initialized");
        }
        System.arraycopy(eyeReliefInMeters, 0, this.eyeReliefInMeters, 0, 2);
        this.distortionSpecs = DistortionSpec.CalculateDistortionSpec(deviceConfig, eyeReliefInMeters);
        this.deviceConfig = deviceConfig;
    }

    // Pow2 for the Morton order to work!
    // 4 is too low - it is easy to see the 'wobbles' in the HMD.
    // 5 is close but you can see pixel differences with even/odd frame checking.
    // 6 is indistinguishable on a monitor on even/odd frames.
    private static final int DMA_GridSizeLog2   = 6;
    private static final int DMA_GridSize       = 1<<DMA_GridSizeLog2;
    private static final int DMA_NumVertsPerEye = (DMA_GridSize+1)*(DMA_GridSize+1);
    private static final int DMA_NumTrisPerEye  = (DMA_GridSize)*(DMA_GridSize)*2;

    private static float[] TransformTanFovSpaceToScreenNDC(final DistortionSpec distortion,
                                                           final float[] tanEyeAngle, final boolean usePolyApprox /*= false*/ ) {
        final float tanEyeAngleRadius = VectorUtil.normVec2(tanEyeAngle);
        float tanEyeAngleDistortedRadius = distortion.lens.DistortionFnInverseApprox ( tanEyeAngleRadius );
        if ( !usePolyApprox ) {
            tanEyeAngleDistortedRadius = distortion.lens.DistortionFnInverse ( tanEyeAngleRadius );
        }
        final float[] vec2Tmp1 = new float[2];
        final float[] tanEyeAngleDistorted;
        if ( tanEyeAngleRadius > 0.0f ) {
            tanEyeAngleDistorted = VectorUtil.scaleVec2(vec2Tmp1, tanEyeAngle, tanEyeAngleDistortedRadius / tanEyeAngleRadius );
        } else {
            tanEyeAngleDistorted = tanEyeAngle;
        }

        final float[] framebufferNDC =
                VectorUtil.addVec2(vec2Tmp1,
                        VectorUtil.divVec2(vec2Tmp1, tanEyeAngleDistorted, distortion.tanEyeAngleScale),
                        distortion.lensCenter );
        return framebufferNDC;
    }

    private static void TransformScreenNDCToTanFovSpaceChroma (final float[] resultR, final float[] resultG, final float[] resultB,
                                                               final DistortionSpec distortion, final float[] framebufferNDC) {
        // Scale to TanHalfFov space, but still distorted.
        final float[] vec2Tmp1 = new float[2];
        final float[] tanEyeAngleDistorted =
                VectorUtil.scaleVec2(vec2Tmp1, VectorUtil.subVec2(vec2Tmp1, framebufferNDC, distortion.lensCenter), distortion.tanEyeAngleScale);
        // Distort.
        final float radiusSquared = ( tanEyeAngleDistorted[0] * tanEyeAngleDistorted[0] )
                            + ( tanEyeAngleDistorted[1] * tanEyeAngleDistorted[1] );
        final float[] distortionScales = distortion.lens.DistortionFnScaleRadiusSquaredChroma (radiusSquared);
        VectorUtil.scaleVec2(resultR, tanEyeAngleDistorted, distortionScales[0]);
        VectorUtil.scaleVec2(resultG, tanEyeAngleDistorted, distortionScales[1]);
        VectorUtil.scaleVec2(resultB, tanEyeAngleDistorted, distortionScales[2]);
    }

    @Override
    public final DistortionMesh create(final EyeParameter eyeParam, final int distortionBits) {
        // Find the mapping from TanAngle space to target NDC space.
        final ScaleAndOffset2D  eyeToSourceNDC = new ScaleAndOffset2D(eyeParam.fovhv);
        final boolean rightEye = 1 == eyeParam.number;

        // When does the fade-to-black edge start? Chosen heuristically.
        final float fadeOutBorderFraction = 0.075f;


        // Populate vertex buffer info
        float xOffset = 0.0f;
        @SuppressWarnings("unused")
        float uOffset = 0.0f;

        if (rightEye)
        {
            xOffset = 1.0f;
            uOffset = 0.5f;
        }

        // First pass - build up raw vertex data.
        final DistortionVertex[] vertices =  new DistortionVertex[DMA_NumVertsPerEye];
        int currentVertex = 0;
        final float[] tanEyeAnglesR = new float[2], tanEyeAnglesG = new float[2], tanEyeAnglesB = new float[2];

        for ( int y = 0; y <= DMA_GridSize; y++ ) {
            for ( int x = 0; x <= DMA_GridSize; x++ ) {
                final float[] pcurVert = new float[DistortionVertex.def_total_size];
                int idx = 0;

                final float[] sourceCoordNDC = new float[2];
                // NDC texture coords [-1,+1]
                sourceCoordNDC[0] = 2.0f * ( (float)x / (float)DMA_GridSize ) - 1.0f;
                sourceCoordNDC[1] = 2.0f * ( (float)y / (float)DMA_GridSize ) - 1.0f;
                final float[] tanEyeAngle = eyeToSourceNDC.convertToTanFovSpace(sourceCoordNDC);

                // This is the function that does the really heavy lifting.
                final float[] screenNDC = TransformTanFovSpaceToScreenNDC ( distortionSpecs[eyeParam.number], tanEyeAngle, false );

                // VIGNETTE - Fade out at texture edges.
                float edgeFadeIn = ( 1.0f / fadeOutBorderFraction ) *
                                   ( 1.0f - Math.max( FloatUtil.abs( sourceCoordNDC[0] ), FloatUtil.abs( sourceCoordNDC[0] ) ) );
                // Also fade out at screen edges.
                final float edgeFadeInScreen = ( 2.0f / fadeOutBorderFraction ) *
                                               ( 1.0f - Math.max( FloatUtil.abs( screenNDC[0] ), FloatUtil.abs( screenNDC[1] ) ) );
                edgeFadeIn = Math.min( edgeFadeInScreen, edgeFadeIn );

                // TIMEWARP
                float timewarpLerp;
                {
                    switch ( deviceConfig.shutterType )
                    {
                    case Global:
                        timewarpLerp = 0.0f;
                        break;
                    case RollingLeftToRight:
                        // Retrace is left to right - left eye goes 0.0 -> 0.5, then right goes 0.5 -> 1.0
                        timewarpLerp = screenNDC[0] * 0.25f + 0.25f;
                        if (rightEye)
                        {
                            timewarpLerp += 0.5f;
                        }
                        break;
                    case RollingRightToLeft:
                        // Retrace is right to left - right eye goes 0.0 -> 0.5, then left goes 0.5 -> 1.0
                        timewarpLerp = 0.75f - screenNDC[0] * 0.25f;
                        if (rightEye)
                        {
                            timewarpLerp -= 0.5f;
                        }
                        break;
                    case RollingTopToBottom:
                        // Retrace is top to bottom on both eyes at the same time.
                        timewarpLerp = screenNDC[1] * 0.5f + 0.5f;
                        break;
                    default:
                        timewarpLerp = 0.0f;
                        break;
                    }

                }

                // We then need RGB UVs. Since chromatic aberration is generated from screen coords, not
                // directly from texture NDCs, we can't just use tanEyeAngle, we need to go the long way round.
                TransformScreenNDCToTanFovSpaceChroma ( tanEyeAnglesR, tanEyeAnglesG, tanEyeAnglesB,
                                                        distortionSpecs[eyeParam.number], screenNDC );

                // Don't let verts overlap to the other eye.
                screenNDC[0] = Math.max ( -1.0f, Math.min( screenNDC[0], 1.0f ) );
                screenNDC[1] = Math.max ( -1.0f, Math.min( screenNDC[1], 1.0f ) );

                // POS
                pcurVert[idx++] = 0.5f * screenNDC[0] - 0.5f + xOffset;
                pcurVert[idx++] = -screenNDC[1];

                // VIGNETTE
                pcurVert[idx++] = Math.max( 0.0f, Math.min( edgeFadeIn, 1.0f ) );

                // TIMEWARP
                pcurVert[idx++] = timewarpLerp;

                // Chroma RGB
                pcurVert[idx++] = tanEyeAnglesR[0];
                pcurVert[idx++] = tanEyeAnglesR[1];
                pcurVert[idx++] = tanEyeAnglesG[0];
                pcurVert[idx++] = tanEyeAnglesG[1];
                pcurVert[idx++] = tanEyeAnglesB[0];
                pcurVert[idx++] = tanEyeAnglesB[1];

                vertices[currentVertex++] = new DistortionVertex(
                        pcurVert, DistortionVertex.def_pos_size,
                        DistortionVertex.def_vignetteFactor_size, DistortionVertex.def_timewarpFactor_size, DistortionVertex.def_texR_size,
                        DistortionVertex.def_texG_size, DistortionVertex.def_texB_size);
            }
        }


        // Populate index buffer info
        final short[] indices = new short[DMA_NumTrisPerEye*3];
        int idx = 0;

        for ( int triNum = 0; triNum < DMA_GridSize * DMA_GridSize; triNum++ )
        {
            // Use a Morton order to help locality of FB, texture and vertex cache.
            // (0.325ms raster order -> 0.257ms Morton order)
            final int x = ( ( triNum & 0x0001 ) >> 0 ) |
                          ( ( triNum & 0x0004 ) >> 1 ) |
                          ( ( triNum & 0x0010 ) >> 2 ) |
                          ( ( triNum & 0x0040 ) >> 3 ) |
                          ( ( triNum & 0x0100 ) >> 4 ) |
                          ( ( triNum & 0x0400 ) >> 5 ) |
                          ( ( triNum & 0x1000 ) >> 6 ) |
                          ( ( triNum & 0x4000 ) >> 7 );

            final int y = ( ( triNum & 0x0002 ) >> 1 ) |
                          ( ( triNum & 0x0008 ) >> 2 ) |
                          ( ( triNum & 0x0020 ) >> 3 ) |
                          ( ( triNum & 0x0080 ) >> 4 ) |
                          ( ( triNum & 0x0200 ) >> 5 ) |
                          ( ( triNum & 0x0800 ) >> 6 ) |
                          ( ( triNum & 0x2000 ) >> 7 ) |
                          ( ( triNum & 0x8000 ) >> 8 );

            final int FirstVertex = x * (DMA_GridSize+1) + y;
            // Another twist - we want the top-left and bottom-right quadrants to
            // have the triangles split one way, the other two split the other.
            // +---+---+---+---+
            // |  /|  /|\  |\  |
            // | / | / | \ | \ |
            // |/  |/  |  \|  \|
            // +---+---+---+---+
            // |  /|  /|\  |\  |
            // | / | / | \ | \ |
            // |/  |/  |  \|  \|
            // +---+---+---+---+
            // |\  |\  |  /|  /|
            // | \ | \ | / | / |
            // |  \|  \|/  |/  |
            // +---+---+---+---+
            // |\  |\  |  /|  /|
            // | \ | \ | / | / |
            // |  \|  \|/  |/  |
            // +---+---+---+---+
            // This way triangle edges don't span long distances over the distortion function,
            // so linear interpolation works better & we can use fewer tris.
            if ( ( x < DMA_GridSize/2 ) != ( y < DMA_GridSize/2 ) )       // != is logical XOR
            {
                indices[idx++] = (short)(FirstVertex);
                indices[idx++] = (short)(FirstVertex+1);
                indices[idx++] = (short)(FirstVertex+(DMA_GridSize+1)+1);

                indices[idx++] = (short)(FirstVertex+(DMA_GridSize+1)+1);
                indices[idx++] = (short)(FirstVertex+(DMA_GridSize+1));
                indices[idx++] = (short)(FirstVertex);
            }
            else
            {
                indices[idx++] = (short)(FirstVertex);
                indices[idx++] = (short)(FirstVertex+1);
                indices[idx++] = (short)(FirstVertex+(DMA_GridSize+1));

                indices[idx++] = (short)(FirstVertex+1);
                indices[idx++] = (short)(FirstVertex+(DMA_GridSize+1)+1);
                indices[idx++] = (short)(FirstVertex+(DMA_GridSize+1));
            }
        }

        return new DistortionMesh(vertices, vertices.length,
                indices, indices.length);
    }

}
