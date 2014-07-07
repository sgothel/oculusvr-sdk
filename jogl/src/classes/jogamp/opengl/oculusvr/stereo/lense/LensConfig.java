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

public class LensConfig {
    public static enum DistortionEquation { RecipPoly4, CatmullRom10 };

    public static final int NumCoefficients = 11;

    private DistortionEquation eqn;
    /* pp */ float MetersPerTanAngleAtCenter;
    private final float[] K;
    private final float[] InvK;

    private float MaxR;       // The highest R you're going to query for - the curve is unpredictable beyond it.
    private float MaxInvR;

    // Additional per-channel scaling is applied after distortion:
    //  Index [0] - Red channel constant coefficient.
    //  Index [1] - Red channel r^2 coefficient.
    //  Index [2] - Blue channel constant coefficient.
    //  Index [3] - Blue channel r^2 coefficient.
    private final float[] ChromaticAberration = new float[4];

    public LensConfig() {
        this.K = new float[NumCoefficients];
        this.InvK = new float[NumCoefficients];
        SetToIdentity();
    }

    public LensConfig(final DistortionEquation eqn, final float MetersPerTanAngleAtCenter, final float[] K) {
        this.K = new float[NumCoefficients];
        this.InvK = new float[NumCoefficients];
        SetToIdentity();
        this.eqn = eqn;
        this.MetersPerTanAngleAtCenter = MetersPerTanAngleAtCenter;
        System.arraycopy(K, 0, this.K, 0, K.length);

        // Chromatic aberration doesn't seem to change with eye relief.
        ChromaticAberration[0]        = -0.006f;
        ChromaticAberration[1]        =  0.0f;
        ChromaticAberration[2]        =  0.014f;
        ChromaticAberration[3]        =  0.0f;
    }

    private void SetUpInverseApprox() {
        switch ( eqn )
        {
            case RecipPoly4: {
                final float[] sampleR = new float[4];
                final float[] sampleRSq = new float[4];
                final float[] sampleInv = new float[4];
                final float[] sampleFit = new float[4];
                final float maxR = MaxInvR;

                // Found heuristically...
                sampleR[0] = 0.0f;
                sampleR[1] = maxR * 0.4f;
                sampleR[2] = maxR * 0.8f;
                sampleR[3] = maxR * 1.5f;
                for ( int i = 0; i < 4; i++ ) {
                    sampleRSq[i] = sampleR[i] * sampleR[i];
                    sampleInv[i] = DistortionFnInverse ( sampleR[i] );
                    sampleFit[i] = sampleR[i] / sampleInv[i];
                }
                sampleFit[0] = 1.0f;
                FitCubicPolynomial ( InvK, sampleRSq, sampleFit );
            }
            break;
            case CatmullRom10: {
                final int NumSegments = NumCoefficients;
                for ( int i = 1; i < NumSegments; i++ ) {
                    final float scaledRsq = i;
                    final float rsq = scaledRsq * MaxInvR * MaxInvR / ( NumSegments - 1);
                    final float r = (float)Math.sqrt ( rsq );
                    final float inv = DistortionFnInverse ( r );
                    InvK[i] = inv / r;
                    InvK[0] = 1.0f;     // TODO: fix this.
                }
            }
            break;
        default:
            throw new InternalError("unsupported EQ "+eqn);
        }

    }

    private void SetToIdentity() {
        for ( int i = 0; i < NumCoefficients; i++ )
        {
            K[i] = 0.0f;
            InvK[i] = 0.0f;
        }
        eqn  = DistortionEquation.RecipPoly4;
        K[0] = 1.0f;
        InvK[0] = 1.0f;
        MaxR = 1.0f;
        MaxInvR = 1.0f;
        ChromaticAberration[0] = 0.0f;
        ChromaticAberration[1] = 0.0f;
        ChromaticAberration[2] = 0.0f;
        ChromaticAberration[3] = 0.0f;
        MetersPerTanAngleAtCenter = 0.05f;
    }

    // DistortionFn applies distortion to the argument.
    // Input: the distance in TanAngle/NIC space from the optical center to the input pixel.
    // Output: the resulting distance after distortion.
    public float DistortionFn(final float r) {
        return r * DistortionFnScaleRadiusSquared ( r * r );
    }

    // The result is a scaling applied to the distance.
    public float DistortionFnInverseApprox(final float r) {
        final float rsq = r * r;
        final float scale;

        switch ( eqn )
        {
            case RecipPoly4: {
                scale = 1.0f / ( InvK[0] + rsq * ( InvK[1] + rsq * ( InvK[2] + rsq * InvK[3] ) ) );
            }
            break;
            case CatmullRom10: {
                // A Catmull-Rom spline through the values 1.0, K[1], K[2] ... K[9]
                // evenly spaced in R^2 from 0.0 to MaxR^2
                // K[0] controls the slope at radius=0.0, rather than the actual value.
                final int NumSegments = NumCoefficients;
                final float scaledRsq = (NumSegments-1) * rsq / ( MaxInvR * MaxInvR );
                scale = EvalCatmullRom10Spline ( InvK, scaledRsq );
            }
            break;
        default:
            throw new InternalError("unsupported EQ "+eqn);
        }
        return r * scale;
    }

    // DistortionFnInverse computes the inverse of the distortion function on an argument.
    public float DistortionFnInverse(final float r) {
        float s, d;
        float delta = r * 0.25f;

        // Better to start guessing too low & take longer to converge than too high
        // and hit singularities. Empirically, r * 0.5f is too high in some cases.
        s = r * 0.25f;
        d = Math.abs(r - DistortionFn(s));

        for (int i = 0; i < 20; i++)
        {
            final float sUp   = s + delta;
            final float sDown = s - delta;
            final float dUp   = Math.abs(r - DistortionFn(sUp));
            final float dDown = Math.abs(r - DistortionFn(sDown));

            if (dUp < d)
            {
                s = sUp;
                d = dUp;
            }
            else if (dDown < d)
            {
                s = sDown;
                d = dDown;
            }
            else
            {
                delta *= 0.5f;
            }
        }
        return s;
    }

    // The result is a scaling applied to the distance from the center of the lens.
    public float DistortionFnScaleRadiusSquared (final float rsq) {
        final float scale;
        switch ( eqn )
        {
            case RecipPoly4: {
                scale = 1.0f / ( K[0] + rsq * ( K[1] + rsq * ( K[2] + rsq * K[3] ) ) );
            }
            break;
            case CatmullRom10: {
                // A Catmull-Rom spline through the values 1.0, K[1], K[2] ... K[10]
                // evenly spaced in R^2 from 0.0 to MaxR^2
                // K[0] controls the slope at radius=0.0, rather than the actual value.
                final int NumSegments = LensConfig.NumCoefficients;
                final float scaledRsq = (NumSegments-1) * rsq / ( MaxR * MaxR );
                scale = EvalCatmullRom10Spline ( K, scaledRsq );
            }
            break;
        default:
            throw new InternalError("unsupported EQ "+eqn);
        }
        return scale;
    }

    // x,y,z components map to r,g,b
    public float[] DistortionFnScaleRadiusSquaredChroma(final float radiusSquared) {
        final float scale = DistortionFnScaleRadiusSquared ( radiusSquared );
        final float[] scaleRGB = new float[3];
        scaleRGB[0] = scale * ( 1.0f + ChromaticAberration[0] + radiusSquared * ChromaticAberration[1] );     // Red
        scaleRGB[1] = scale;                                                                                  // Green
        scaleRGB[2] = scale * ( 1.0f + ChromaticAberration[2] + radiusSquared * ChromaticAberration[3] );     // Blue
        return scaleRGB;
    }

    /**
    // Inputs are 4 points (pFitX[0],pFitY[0]) through (pFitX[3],pFitY[3])
    // Result is four coefficients in pResults[0] through pResults[3] such that
    //      y = pResult[0] + x * ( pResult[1] + x * ( pResult[2] + x * ( pResult[3] ) ) );
    // passes through all four input points.
    // Return is true if it succeeded, false if it failed (because two control points
    // have the same pFitX value).
     *
     * @param pResult
     * @param pFitX
     * @param pFitY
     * @return
     */
    private static boolean FitCubicPolynomial ( final float[/*4*/] pResult, final float[/*4*/] pFitX, final float[/*4*/] pFitY ) {
        final float d0 = ( ( pFitX[0]-pFitX[1] ) * ( pFitX[0]-pFitX[2] ) * ( pFitX[0]-pFitX[3] ) );
        final float d1 = ( ( pFitX[1]-pFitX[2] ) * ( pFitX[1]-pFitX[3] ) * ( pFitX[1]-pFitX[0] ) );
        final float d2 = ( ( pFitX[2]-pFitX[3] ) * ( pFitX[2]-pFitX[0] ) * ( pFitX[2]-pFitX[1] ) );
        final float d3 = ( ( pFitX[3]-pFitX[0] ) * ( pFitX[3]-pFitX[1] ) * ( pFitX[3]-pFitX[2] ) );

        if ( ( d0 == 0.0f ) || ( d1 == 0.0f ) || ( d2 == 0.0f ) || ( d3 == 0.0f ) )
        {
            return false;
        }

        final float f0 = pFitY[0] / d0;
        final float f1 = pFitY[1] / d1;
        final float f2 = pFitY[2] / d2;
        final float f3 = pFitY[3] / d3;

        pResult[0] = -( f0*pFitX[1]*pFitX[2]*pFitX[3]
                      + f1*pFitX[0]*pFitX[2]*pFitX[3]
                      + f2*pFitX[0]*pFitX[1]*pFitX[3]
                      + f3*pFitX[0]*pFitX[1]*pFitX[2] );
        pResult[1] = f0*(pFitX[1]*pFitX[2] + pFitX[2]*pFitX[3] + pFitX[3]*pFitX[1])
                   + f1*(pFitX[0]*pFitX[2] + pFitX[2]*pFitX[3] + pFitX[3]*pFitX[0])
                   + f2*(pFitX[0]*pFitX[1] + pFitX[1]*pFitX[3] + pFitX[3]*pFitX[0])
                   + f3*(pFitX[0]*pFitX[1] + pFitX[1]*pFitX[2] + pFitX[2]*pFitX[0]);
        pResult[2] = -( f0*(pFitX[1]+pFitX[2]+pFitX[3])
                      + f1*(pFitX[0]+pFitX[2]+pFitX[3])
                      + f2*(pFitX[0]+pFitX[1]+pFitX[3])
                      + f3*(pFitX[0]+pFitX[1]+pFitX[2]) );
        pResult[3] = f0 + f1 + f2 + f3;

        return true;
    }

    private static float EvalCatmullRom10Spline ( final float[] K, final float scaledVal ) {
        final int NumSegments = NumCoefficients;

        float scaledValFloor = (float)Math.floor( scaledVal );
        scaledValFloor = Math.max( 0.0f, Math.min( NumSegments-1, scaledValFloor ) );
        final float t = scaledVal - scaledValFloor;
        final int k = (int)scaledValFloor;

        float p0, p1;
        float m0, m1;
        switch ( k )
        {
        case 0:
            // Curve starts at 1.0 with gradient K[1]-K[0]
            p0 = 1.0f;
            m0 =        ( K[1] - K[0] );    // general case would have been (K[1]-K[-1])/2
            p1 = K[1];
            m1 = 0.5f * ( K[2] - K[0] );
            break;
        default:
            // General case
            p0 = K[k  ];
            m0 = 0.5f * ( K[k+1] - K[k-1] );
            p1 = K[k+1];
            m1 = 0.5f * ( K[k+2] - K[k  ] );
            break;
        case NumSegments-2:
            // Last tangent is just the slope of the last two points.
            p0 = K[NumSegments-2];
            m0 = 0.5f * ( K[NumSegments-1] - K[NumSegments-2] );
            p1 = K[NumSegments-1];
            m1 = K[NumSegments-1] - K[NumSegments-2];
            break;
        case NumSegments-1:
            // Beyond the last segment it's just a straight line
            p0 = K[NumSegments-1];
            m0 = K[NumSegments-1] - K[NumSegments-2];
            p1 = p0 + m0;
            m1 = m0;
            break;
        }

        final float omt = 1.0f - t;
        final float res  = ( p0 * ( 1.0f + 2.0f *   t ) + m0 *   t ) * omt * omt
                + ( p1 * ( 1.0f + 2.0f * omt ) - m1 * omt ) *   t *   t;

        return res;
    }

    /** FIXME: Add 'pluggable' lense configuration */
    public static LensConfig[] GenerateLensConfigFromEyeRelief(final float[] eyeReliefInMeters, final DistortionEquation eqn) {
        final LensConfig[] result = new LensConfig[2];
        final DistortionDescriptor[] distortions = LensConfig.CreateDistortionDescriptorsforOVRDK1_CupsABC();
        result[0] = GenerateLensConfigFromEyeRelief(eyeReliefInMeters[0], distortions, eqn);
        result[1] = GenerateLensConfigFromEyeRelief(eyeReliefInMeters[1], distortions, eqn);
        return result;
    }

    private static LensConfig GenerateLensConfigFromEyeRelief(final float eyeReliefInMeters, final DistortionDescriptor[] distortions, final DistortionEquation eqn) {
        final int numDistortions = distortions.length;
        final int defaultDistortion = 0; // index of the default distortion curve to use if zero eye relief supplied

        DistortionDescriptor pUpper = null;
        DistortionDescriptor pLower = null;
        float lerpVal = 0.0f;
        if (eyeReliefInMeters == 0)
        {   // Use a constant default distortion if an invalid eye-relief is supplied
            pLower = distortions[defaultDistortion];
            pUpper = distortions[defaultDistortion];
            lerpVal = 0.0f;
        } else {
            for ( int i = 0; i < numDistortions-1; i++ )
            {
                assert( distortions[i].eyeRelief < distortions[i+1].eyeRelief );
                if ( ( distortions[i].eyeRelief <= eyeReliefInMeters ) && ( distortions[i+1].eyeRelief > eyeReliefInMeters ) )
                {
                    pLower = distortions[i];
                    pUpper = distortions[i+1];
                    lerpVal = ( eyeReliefInMeters - pLower.eyeRelief ) / ( pUpper.eyeRelief - pLower.eyeRelief );
                    // No break here - I want the ASSERT to check everything every time!
                }
            }
        }

        if ( pUpper == null )
        {
            // Do not extrapolate, just clamp - slightly worried about people putting in bogus settings.
            if ( distortions[0].eyeRelief > eyeReliefInMeters )
            {
                pLower = distortions[0];
                pUpper = distortions[0];
            }
            else
            {
                assert ( distortions[numDistortions-1].eyeRelief <= eyeReliefInMeters );
                pLower = distortions[numDistortions-1];
                pUpper = distortions[numDistortions-1];
            }
            lerpVal = 0.0f;
        }
        final float invLerpVal = 1.0f - lerpVal;

        pLower.config.MaxR = pLower.maxRadius;
        pUpper.config.MaxR = pUpper.maxRadius;

        final LensConfig result = new LensConfig();
        // Where is the edge of the lens - no point modelling further than this.
        final float maxValidRadius = invLerpVal * pLower.maxRadius + lerpVal * pUpper.maxRadius;
        result.MaxR = maxValidRadius;

        switch ( eqn )
        {
            case RecipPoly4:{
                    // Lerp control points and fit an equation to them.
                    final float[] fitX = new float[4];
                    final float[] fitY = new float[4];
                    fitX[0] = 0.0f;
                    fitY[0] = 1.0f;
                    for ( int ctrlPt = 1; ctrlPt < 4; ctrlPt ++ )
                    {
                        final float radiusLerp = invLerpVal * pLower.sampleRadius[ctrlPt-1] + lerpVal * pUpper.sampleRadius[ctrlPt-1];
                        final float radiusLerpSq = radiusLerp * radiusLerp;
                        final float fitYLower = pLower.config.DistortionFnScaleRadiusSquared ( radiusLerpSq );
                        final float fitYUpper = pUpper.config.DistortionFnScaleRadiusSquared ( radiusLerpSq );
                        fitX[ctrlPt] = radiusLerpSq;
                        fitY[ctrlPt] = 1.0f / ( invLerpVal * fitYLower + lerpVal * fitYUpper );
                    }
                    result.eqn = DistortionEquation.RecipPoly4;
                    final boolean bSuccess = LensConfig.FitCubicPolynomial ( result.K, fitX, fitY );
                    assert ( bSuccess );

                    // Set up the fast inverse.
                    final float maxRDist = result.DistortionFn ( maxValidRadius );
                    result.MaxInvR = maxRDist;
                    result.SetUpInverseApprox();
                }
                break;
            case CatmullRom10: {
                // Evenly sample & lerp points on the curve.
                final int NumSegments = LensConfig.NumCoefficients;
                result.MaxR = maxValidRadius;
                // Directly interpolate the K0 values
                result.K[0] = invLerpVal * pLower.config.K[0] + lerpVal * pUpper.config.K[0];

                // Sample and interpolate the distortion curves to derive K[1] ... K[n]
                for ( int ctrlPt = 1; ctrlPt < NumSegments; ctrlPt++ )
                {
                    final float radiusSq = ( (float)ctrlPt / (float)(NumSegments-1) ) * maxValidRadius * maxValidRadius;
                    final float fitYLower = pLower.config.DistortionFnScaleRadiusSquared ( radiusSq );
                    final float fitYUpper = pUpper.config.DistortionFnScaleRadiusSquared ( radiusSq );
                    final float fitLerp = invLerpVal * fitYLower + lerpVal * fitYUpper;
                    result.K[ctrlPt] = fitLerp;
                }

                result.eqn = DistortionEquation.CatmullRom10;

                for ( int ctrlPt = 1; ctrlPt < NumSegments; ctrlPt++ )
                {
                    final float radiusSq = ( (float)ctrlPt / (float)(NumSegments-1) ) * maxValidRadius * maxValidRadius;
                    final float val = result.DistortionFnScaleRadiusSquared ( radiusSq );
                    assert ( Math.abs( val - result.K[ctrlPt] ) < 0.0001f );
                }

                // Set up the fast inverse.
                final float maxRDist = result.DistortionFn ( maxValidRadius );
                result.MaxInvR = maxRDist;
                result.SetUpInverseApprox();
            }
            break;
        default:
            throw new InternalError("unsupported EQ "+eqn);
        }

        // Chromatic aberration.
        result.ChromaticAberration[0] = invLerpVal * pLower.config.ChromaticAberration[0] + lerpVal * pUpper.config.ChromaticAberration[0];
        result.ChromaticAberration[1] = invLerpVal * pLower.config.ChromaticAberration[1] + lerpVal * pUpper.config.ChromaticAberration[1];
        result.ChromaticAberration[2] = invLerpVal * pLower.config.ChromaticAberration[2] + lerpVal * pUpper.config.ChromaticAberration[2];
        result.ChromaticAberration[3] = invLerpVal * pLower.config.ChromaticAberration[3] + lerpVal * pUpper.config.ChromaticAberration[3];

        // Scale.
        result.MetersPerTanAngleAtCenter =  pLower.config.MetersPerTanAngleAtCenter * invLerpVal +
                                            pUpper.config.MetersPerTanAngleAtCenter * lerpVal;

        return result;
    }

    public static class DistortionDescriptor {
        public DistortionDescriptor(final LensConfig lens, final float eyeRelief,
                                    final float[] sampleRadius, final float maxRadius) {
            this.config = lens;
            this.eyeRelief = eyeRelief;
            this.sampleRadius = sampleRadius;
            this.maxRadius = maxRadius;
        }

        final LensConfig config;
        final float eyeRelief;
        final float[] sampleRadius;
        final float maxRadius;
    }

    /*** Hardcoded OculusVR DK1 A, B, C eye cups (lenses) */
    public static DistortionDescriptor[] CreateDistortionDescriptorsforOVRDK1_CupsABC() {
        return new DistortionDescriptor[] {
            // Tuned at minimum dial setting - extended to r^2 == 1.8
            new DistortionDescriptor(
                    new LensConfig(DistortionEquation.CatmullRom10,
                                   0.0425f, // MetersPerTanAngleAtCenter
                                   new float[] { 1.0000f,   // K00
                                                 1.06505f,  // K01
                                                 1.14725f,  // K02
                                                 1.2705f,   // K03
                                                 1.48f,     // K04
                                                 1.87f,     // K05
                                                 2.534f,    // K06
                                                 3.6f,      // K07
                                                 5.1f,      // K08
                                                 7.4f,      // K09
                                                 11.0f} ),  // K10
                    0.012760465f - 0.005f, // eyeRelief
                    new float[] { 0.222717149f, 0.512249443f, 0.712694878f }, // sampleRadius
                    (float)Math.sqrt(1.8f) ), // maxRadius
            // Tuned at middle dial setting
            new DistortionDescriptor(
                    new LensConfig(DistortionEquation.CatmullRom10,
                                   0.0425f, // MetersPerTanAngleAtCenter
                                   new float[] { 1.0000f,       // K00
                                                 1.032407264f,  // K01
                                                 1.07160462f,   // K02
                                                 1.11998388f,   // K03
                                                 1.1808606f,    // K04
                                                 1.2590494f,    // K05
                                                 1.361915f,     // K06
                                                 1.5014339f,    // K07
                                                 1.6986004f,    // K08
                                                 1.9940577f,    // K09
                                                 2.4783147f} ), // K10
                    0.012760465f, // eyeRelief
                    new float[] { 0.222717149f, 0.512249443f, 0.712694878f }, // sampleRadius
                    1.0f ), // maxRadius
            // Tuned at maximum dial setting
            new DistortionDescriptor(
                    new LensConfig(DistortionEquation.CatmullRom10,
                                   0.0425f, // MetersPerTanAngleAtCenter
                                   new float[] { 1.0102f,       // K00
                                                 1.0371f,  // K01
                                                 1.0831f,   // K02
                                                 1.1353f,   // K03
                                                 1.2f,    // K04
                                                 1.2851f,    // K05
                                                 1.3979f,     // K06
                                                 1.56f,    // K07
                                                 1.8f,    // K08
                                                 2.25f,    // K09
                                                 3.0f} ), // K10
                    0.012760465f + 0.005f, // eyeRelief
                    new float[] { 0.222717149f, 0.512249443f, 0.712694878f }, // sampleRadius
                    1.0f ), // maxRadius
        };
    }

}