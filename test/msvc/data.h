/**
 * @author 黄艺华 (lernanto@foxmail.com)
 * @brief 测试线性回归的数据
 */

#ifndef _DATA_H
#define _DATA_H

#include "arm_math.h"


#define TRAIN_SAMPLE_NUM  100
#define TEST_SAMPLE_NUM 10
#define FEATURE_DIM 10
#define TARGET_DIM  5

static const float32_t _coef[TARGET_DIM][FEATURE_DIM] = {
    { 3.938638f, 1.227211f, 0.878373f, 2.061817f, 1.316537f, -0.202576f, 1.147425f, 1.126924f, 2.161004f, 3.312388f },
    { 1.066629f, 3.987960f, 0.217437f, 1.205825f, 0.582624f, -0.687164f, -2.402328f, 1.766778f, 2.483601f, -0.706291f },
    { 3.430056f, 3.822188f, 3.605134f, 0.697245f, 4.583991f, 0.548405f, 4.536303f, 2.040515f, 2.494647f, 3.820229f },
    { 1.015167f, 0.798288f, 1.702466f, 0.971349f, 0.866755f, 0.318890f, 3.390288f, 6.429464f, 3.432255f, 0.054261f },
    { 3.413986f, -0.283090f, 0.305046f, -1.420904f, 3.328033f, 1.781775f, 3.474160f, 1.210643f, 3.850190f, -1.794156f },
};

static const float32_t train_x[TRAIN_SAMPLE_NUM][FEATURE_DIM] = {
    { 1.000000f, -0.784401f, 0.279857f, -0.405508f, -0.489702f, 1.169653f, -1.949645f, 1.412790f, 1.274143f, 1.223228f },
    { 1.000000f, -1.196420f, 0.208188f, -1.747398f, 2.881145f, -2.173840f, -1.788700f, -0.889847f, 0.267260f, -0.866100f },
    { 1.000000f, 0.754583f, -0.050806f, 0.535152f, 0.651235f, -1.300247f, 0.883387f, -0.978392f, 0.688882f, -0.476097f },
    { 1.000000f, 0.358787f, 0.764504f, -1.203256f, 0.097967f, -0.043595f, 0.574320f, 1.030990f, 0.793297f, 2.649915f },
    { 1.000000f, 1.728930f, 0.635585f, 0.018069f, -0.065060f, -0.114137f, 0.996810f, -0.173009f, -1.013857f, -0.240874f },
    { 1.000000f, -0.573159f, 0.255695f, 0.401555f, 0.945499f, -2.484196f, -0.505972f, -0.597311f, 0.194218f, 1.592003f },
    { 1.000000f, -0.333306f, 1.299817f, -1.220840f, -1.708286f, -0.756518f, 1.894884f, -0.447499f, 0.791912f, -0.109377f },
    { 1.000000f, 0.092809f, 0.414816f, -0.841970f, 0.117992f, -0.186917f, -0.068145f, -0.088426f, -0.767445f, -0.339579f },
    { 1.000000f, -1.699239f, -1.677888f, -0.086750f, -0.461296f, 1.020359f, -1.383486f, -0.234695f, -0.017989f, 2.186439f },
    { 1.000000f, 0.642851f, 0.491835f, -0.159125f, 1.610570f, -1.946712f, 0.329093f, -1.289229f, -0.606255f, 0.533219f },
    { 1.000000f, -1.037114f, 0.071531f, 0.447268f, 0.273201f, 0.802127f, -0.538736f, -0.193954f, 1.911630f, 0.107582f },
    { 1.000000f, -1.212687f, -0.104420f, -0.780036f, 0.566687f, -1.198118f, 0.760809f, -0.212219f, 0.393469f, -0.118506f },
    { 1.000000f, 1.990393f, -0.633772f, 1.441209f, 0.154171f, 0.363661f, 0.854593f, 2.350579f, -1.266879f, -0.161766f },
    { 1.000000f, 0.292059f, -0.286356f, -2.848301f, 1.069347f, -0.489342f, -0.122975f, 0.850794f, -0.965673f, 1.773895f },
    { 1.000000f, 0.366402f, 0.932277f, 1.301330f, -0.023841f, -1.434269f, 0.471230f, 1.292301f, -1.339922f, -1.534070f },
    { 1.000000f, -0.132924f, 0.902537f, -1.008148f, -1.036104f, -0.505371f, -1.284181f, 0.853546f, -1.602293f, -0.249714f },
    { 1.000000f, 1.390104f, 0.278442f, 1.485529f, -0.230705f, -0.075346f, 0.746452f, -1.605434f, -1.326671f, -1.356059f },
    { 1.000000f, -0.306667f, -1.266624f, -0.076056f, 2.864647f, -0.525238f, 0.180134f, -0.375523f, -0.862538f, 1.669495f },
    { 1.000000f, -1.003933f, -1.030699f, -1.698333f, 0.609746f, 0.309962f, -1.540887f, -1.097496f, 1.066524f, -1.299105f },
    { 1.000000f, -0.162250f, -0.511661f, 0.131371f, 0.006753f, -0.857713f, -1.457122f, -0.553702f, 0.455681f, -0.416953f },
    { 1.000000f, 2.409840f, 0.957937f, -0.176755f, 1.277859f, 1.449239f, -0.073834f, 0.366185f, 0.716954f, -1.214559f },
    { 1.000000f, 0.189744f, -0.672600f, 0.594803f, 2.039141f, 0.441607f, -0.230776f, -0.768643f, -0.046597f, 0.582088f },
    { 1.000000f, -1.145343f, 0.559017f, 0.698190f, 0.954591f, 0.366266f, 0.492305f, 0.486617f, 1.143676f, -0.337541f },
    { 1.000000f, -1.284425f, 0.421483f, 0.880354f, 0.103541f, -0.149944f, 1.064321f, -0.758491f, 0.942101f, -1.410738f },
    { 1.000000f, -0.005231f, 0.734231f, 0.509777f, 0.372144f, -1.893861f, -0.396770f, -0.157462f, 0.240471f, 0.155875f },
    { 1.000000f, 0.019833f, 0.151441f, 2.075139f, -1.024076f, 0.874522f, 1.437017f, -0.866298f, -0.283547f, 0.021560f },
    { 1.000000f, -0.502163f, 1.336190f, 1.993837f, -0.246864f, -0.085040f, -1.375457f, -2.105644f, 0.974257f, -1.113763f },
    { 1.000000f, -3.016643f, 0.599969f, -1.279970f, -0.689370f, -0.008440f, 0.432044f, 0.353219f, 1.310567f, -0.819680f },
    { 1.000000f, 1.165680f, 2.653251f, 0.750441f, 1.361256f, -0.315687f, 0.016150f, 0.467146f, 0.692982f, -0.112423f },
    { 1.000000f, 0.710707f, -2.641781f, 0.589511f, 0.823908f, -0.019574f, 0.455073f, -0.230101f, 1.060524f, 0.841601f },
    { 1.000000f, 0.292498f, -0.304733f, -1.463498f, 0.556734f, -0.143361f, -1.728086f, -0.029984f, -0.882593f, -0.896818f },
    { 1.000000f, 1.064016f, 1.736503f, 0.241655f, 0.377051f, 0.009776f, -0.782877f, -1.854764f, 1.717699f, -1.141355f },
    { 1.000000f, 0.686414f, -0.280615f, 0.180522f, -1.608330f, -0.520721f, 0.268201f, 0.818346f, 0.207831f, -0.796115f },
    { 1.000000f, 0.510984f, 1.165274f, -0.648907f, -0.152935f, 0.033119f, -1.180137f, -0.113454f, 0.068983f, 1.117627f },
    { 1.000000f, 0.033694f, 0.623947f, 0.176008f, 2.671822f, 0.185227f, -0.637001f, -0.977226f, -0.153212f, 0.628686f },
    { 1.000000f, 1.730768f, 0.997676f, 0.151552f, 0.863133f, 1.167696f, 0.049768f, -0.675472f, -0.246611f, -0.134507f },
    { 1.000000f, -1.121933f, -1.208204f, 0.907226f, 1.716468f, -1.258127f, -0.607579f, 0.817085f, -0.121657f, -0.006263f },
    { 1.000000f, 0.192105f, 1.371952f, 0.181944f, 0.599952f, 0.301381f, -0.730153f, -0.124095f, 2.357094f, -0.937532f },
    { 1.000000f, 0.982818f, 0.345511f, 1.132031f, 1.211392f, 0.081554f, -0.941027f, 0.697784f, 0.064041f, 1.153220f },
    { 1.000000f, 0.861566f, 1.012181f, 1.703199f, 0.591577f, -0.183221f, -1.187660f, -1.162286f, -0.290467f, 0.569868f },
    { 1.000000f, -0.785182f, -0.048259f, 0.949196f, -0.307154f, -1.564526f, -0.466350f, -0.565453f, -0.736705f, 1.577734f },
    { 1.000000f, -0.911773f, -0.138569f, 0.315263f, 1.123167f, 1.893706f, 1.129690f, 1.688737f, -0.823974f, 0.512451f },
    { 1.000000f, -1.135622f, 0.416718f, -0.399906f, 1.450686f, -0.073572f, 1.131297f, -1.427188f, -0.855860f, 1.950631f },
    { 1.000000f, 0.488585f, -0.960018f, -0.541771f, 0.824075f, -0.471440f, -0.408259f, -0.342652f, -0.013543f, -1.604351f },
    { 1.000000f, -0.621510f, -0.154630f, -1.321198f, 0.180617f, -0.245897f, -0.757569f, -1.952484f, -1.178229f, -0.804337f },
    { 1.000000f, -0.167701f, -0.349600f, 1.254768f, -1.234106f, 0.094267f, -1.839943f, 1.706060f, 0.962118f, -0.502387f },
    { 1.000000f, -0.417686f, 1.610119f, -0.087023f, 0.399320f, -0.011078f, -0.337829f, -0.451329f, 0.935839f, 1.462821f },
    { 1.000000f, 0.430287f, -0.988499f, 1.319199f, 1.154303f, 0.137636f, 0.103098f, -0.622504f, -0.174421f, 0.355442f },
    { 1.000000f, -0.867439f, -0.160688f, 0.740748f, 0.499510f, -0.648295f, 0.286929f, -0.082930f, -1.915555f, 0.412538f },
    { 1.000000f, 0.763087f, -0.574052f, -0.119652f, -0.608781f, -0.562858f, 0.126883f, -0.868970f, 1.097767f, 0.407043f },
    { 1.000000f, -0.061154f, -0.450969f, 0.517185f, -0.288620f, -1.050294f, -0.333722f, -0.336298f, -1.451725f, 0.980547f },
    { 1.000000f, -0.461440f, -0.377927f, -1.592051f, -0.036014f, -0.289200f, 0.725205f, -0.167084f, 0.316686f, 0.434512f },
    { 1.000000f, -0.613700f, -0.791512f, -0.750815f, -0.820110f, 2.119526f, 1.009506f, -0.215959f, -0.376419f, -0.863569f },
    { 1.000000f, -0.814618f, 0.244698f, -0.742144f, -0.307452f, -1.866116f, 1.400257f, 0.195464f, 0.176690f, 0.996805f },
    { 1.000000f, -1.467742f, -0.821949f, 1.324413f, -1.844267f, -2.392642f, -0.726252f, -0.856109f, -0.978978f, -1.293415f },
    { 1.000000f, 0.885610f, 1.122405f, -0.115909f, 0.950008f, -0.227772f, -1.213689f, -1.506417f, 0.150524f, -0.220721f },
    { 1.000000f, -0.777690f, -0.290567f, -0.377713f, 0.439678f, 2.396885f, 0.703669f, -1.425111f, -0.214378f, 0.993589f },
    { 1.000000f, 1.077355f, -0.665031f, -0.634820f, -0.064845f, 1.114678f, -0.413422f, 0.044308f, 0.583250f, -1.832949f },
    { 1.000000f, 0.472248f, -0.535730f, -2.137994f, -0.627658f, -0.513777f, -0.868766f, 0.872017f, -0.146887f, 0.421670f },
    { 1.000000f, -2.069458f, -0.687053f, -0.646847f, -2.083239f, 0.200807f, 1.164597f, -1.431567f, 0.284595f, -0.654369f },
    { 1.000000f, -0.536151f, -0.678925f, -1.067592f, 0.220951f, -0.388850f, -0.338525f, 1.682396f, -1.441870f, 0.061607f },
    { 1.000000f, -1.312867f, -1.526862f, 0.638076f, -0.426676f, 0.058342f, 0.595167f, 1.650067f, 1.184625f, -0.185197f },
    { 1.000000f, -1.321876f, 0.187079f, 0.861531f, 2.549801f, 1.376813f, 0.186959f, 0.542354f, 0.076275f, -0.112332f },
    { 1.000000f, 0.119564f, 1.636667f, 1.826230f, 0.639948f, 0.749597f, 1.429509f, 1.005573f, -0.844288f, 0.015095f },
    { 1.000000f, 0.119191f, -0.067074f, -0.191171f, 0.322680f, -0.709028f, -0.834151f, 0.173949f, 1.094071f, -0.511623f },
    { 1.000000f, -1.132913f, -1.895341f, 0.274949f, -1.282976f, 0.376115f, 1.467172f, -0.668223f, -0.876196f, 0.762598f },
    { 1.000000f, -1.179848f, 0.197783f, -2.487820f, -0.379527f, 2.266291f, 1.678892f, -0.653541f, -2.132154f, -1.563681f },
    { 1.000000f, -0.905116f, 0.357285f, -1.129638f, -0.777957f, 0.340390f, -1.008876f, 1.913200f, -0.436690f, -1.041488f },
    { 1.000000f, 1.537234f, -0.098824f, 0.685508f, -1.514039f, 1.012209f, -1.034062f, -1.365324f, -1.281557f, 0.054346f },
    { 1.000000f, 0.186392f, -0.246034f, 1.464782f, 0.273758f, 0.048838f, -1.033342f, -0.285195f, -0.499769f, -1.351877f },
    { 1.000000f, -0.175511f, 0.059282f, -1.081202f, -2.361291f, 2.205049f, -0.477803f, -0.296608f, 0.058673f, -0.626706f },
    { 1.000000f, 1.995163f, 0.094261f, 0.249046f, -1.345996f, -1.598158f, -0.641723f, -0.535200f, 0.667976f, -0.588740f },
    { 1.000000f, 0.445056f, -1.276665f, 1.063747f, 0.760799f, -1.228960f, -0.717135f, 0.417407f, 0.169191f, -0.278138f },
    { 1.000000f, 0.449818f, -0.302901f, 1.791631f, 0.519927f, 0.180511f, -0.832907f, 0.400383f, -0.487311f, -1.316525f },
    { 1.000000f, -0.802035f, 0.641083f, -0.306308f, 2.554017f, 0.199385f, -0.761563f, 1.550454f, -0.027065f, 0.197902f },
    { 1.000000f, 0.489713f, -0.617173f, 0.047760f, -1.176208f, 1.086034f, -0.415345f, -0.322530f, 2.177354f, 0.764813f },
    { 1.000000f, 0.264947f, -0.552337f, -0.796724f, 0.828488f, 0.696526f, -0.914866f, -1.121097f, 0.467282f, -1.144723f },
    { 1.000000f, -0.341291f, -0.241712f, -0.331549f, -1.029702f, 1.016840f, 0.204266f, -0.754541f, -1.401233f, -0.874092f },
    { 1.000000f, -1.470496f, 0.766107f, -0.258266f, -0.115949f, -0.163866f, 0.919856f, -0.756451f, -0.489345f, -0.960652f },
    { 1.000000f, 1.247650f, -1.977531f, 0.844404f, 0.915229f, 0.342666f, -0.164954f, -0.647125f, 1.000263f, -0.672426f },
    { 1.000000f, -1.067944f, -0.261315f, -0.495767f, -0.514272f, 0.819973f, -1.102170f, 0.125178f, 0.533813f, -0.209823f },
    { 1.000000f, -0.055442f, 2.168959f, -0.065902f, -1.036481f, 0.944811f, 0.171228f, -1.345450f, -2.327304f, -2.501815f },
    { 1.000000f, -1.057872f, -0.268188f, -0.483013f, 0.868263f, -0.154074f, 0.108033f, 0.104707f, -1.626391f, -0.486621f },
    { 1.000000f, -0.759315f, 0.217397f, 0.017651f, 1.060211f, -0.872049f, 0.485556f, 1.606194f, 0.058562f, 0.070215f },
    { 1.000000f, -0.001985f, -0.784237f, 0.277862f, -1.053546f, -1.184970f, -0.382585f, 0.124322f, 0.806137f, -0.073775f },
    { 1.000000f, -0.310215f, -0.514905f, 0.381514f, -0.497774f, -0.091275f, -0.074246f, -0.443134f, -0.011984f, 0.663848f },
    { 1.000000f, -1.141989f, -0.300164f, 0.261335f, -0.891901f, -2.127914f, 0.593283f, -0.437048f, -0.635557f, -0.881436f },
    { 1.000000f, -0.646804f, 0.433902f, 0.295558f, 0.355637f, 0.768074f, -0.031609f, 1.399922f, 0.817948f, -0.195873f },
    { 1.000000f, 0.047326f, -0.357253f, -1.392415f, -0.581902f, 0.032955f, 0.539195f, -1.245377f, 0.924502f, 0.141089f },
    { 1.000000f, -0.174933f, -0.959284f, -0.401824f, -0.870299f, 0.442232f, 0.205112f, 0.279879f, -2.264124f, 0.518001f },
    { 1.000000f, 0.158224f, 1.097523f, 0.541180f, 2.216560f, 0.920708f, -0.086800f, -1.418269f, 2.063066f, -0.579499f },
    { 1.000000f, -0.007072f, 1.948590f, -0.038507f, -1.766976f, 0.446410f, 1.226311f, 0.451186f, -0.036000f, 1.683201f },
    { 1.000000f, 0.058725f, 0.578279f, -1.774965f, 0.536165f, -1.925203f, 1.326886f, -0.054507f, 1.622175f, 1.258286f },
    { 1.000000f, -0.828674f, 0.794312f, -0.271192f, 2.012496f, -1.108221f, 0.788385f, -1.115842f, 1.610356f, 0.947461f },
    { 1.000000f, -0.611761f, -1.175381f, 0.831216f, -0.718265f, -1.810881f, -0.343272f, 0.302039f, 0.993431f, -0.257902f },
    { 1.000000f, -0.131450f, -0.236757f, 0.553340f, 0.453326f, -0.934586f, -0.164101f, 0.855252f, -0.016140f, 0.814033f },
    { 1.000000f, -0.012734f, 0.349872f, 1.105159f, -2.539464f, -0.748621f, 0.581826f, -0.454616f, 0.307146f, 0.795492f },
    { 1.000000f, 1.324324f, -1.160051f, 1.860421f, -2.608826f, 1.110698f, -1.144054f, -0.127420f, 0.422272f, -0.826847f },
    { 1.000000f, 1.238812f, 1.210582f, 0.349337f, 1.053031f, 0.994262f, 2.096698f, -1.574764f, -0.757427f, -1.422968f },
    { 1.000000f, 0.980603f, 0.506503f, -0.290777f, 0.082973f, -0.086528f, 1.438779f, 0.075661f, 0.104789f, 0.156464f },
};

static float32_t train_y[TRAIN_SAMPLE_NUM][TARGET_DIM] = {
    { 8.278260f, 6.469982f, 1.125084f, 5.536048f, 2.875941f },
    { -3.013253f, 0.852789f, -1.387697f, -9.368862f, 8.159895f },
    { 6.688184f, 4.416195f, 9.881161f, 2.577122f, 7.129844f },
    { 15.500402f, 2.585907f, 21.823861f, 12.724782f, 7.456804f },
    { 3.784640f, 1.494135f, 11.269149f, 1.950292f, 3.266090f },
    { 9.564030f, 3.243086f, 8.080191f, -4.676054f, -2.087467f },
    { 2.963589f, -3.337254f, 6.134907f, 6.225809f, 8.029959f },
    { 0.393391f, -0.585586f, 2.941481f, -3.975601f, 1.900595f },
    { 6.749493f, -5.051113f, -7.435706f, -9.892367f, -6.249517f },
    { 5.606434f, -0.661086f, 12.965645f, -6.625687f, 1.968064f },
    { 8.063377f, 2.459377f, 3.802454f, 5.242684f, 11.728701f },
    { 1.393995f, -3.415389f, 3.850069f, 0.898223f, 8.803341f },
    { 8.884985f, 8.351162f, 15.236731f, 17.643543f, 3.841719f },
    { 2.431600f, -1.781005f, 12.457718f, 0.019754f, 3.125559f },
    { 1.288904f, 4.460435f, 5.325591f, 9.542770f, 0.194555f },
    { -2.172336f, 0.888940f, -8.325037f, -4.445714f, -9.774217f },
    { 1.080699f, 0.223751f, 1.289004f, -8.050818f, -3.794698f },
    { 10.030510f, -2.126566f, 12.364677f, -4.997947f, 4.045512f },
    { -7.191637f, 0.333714f, -13.201932f, -11.613527f, 9.647713f },
    { 1.785179f, 4.789915f, -6.384094f, -6.698792f, -1.755172f },
    { 6.517975f, 12.972607f, 18.724034f, 12.042982f, 13.790095f },
    { 7.821981f, 3.621960f, 10.932726f, -3.777063f, 7.885882f },
    { 8.791829f, 1.224193f, 11.305677f, 10.971883f, 15.800022f },
    { 2.048977f, -4.501441f, 0.733313f, 3.370370f, 11.240401f },
    { 8.218707f, 5.256687f, 7.786363f, 0.808238f, 0.245508f },
    { 5.738932f, -3.743607f, 6.012837f, 1.110691f, 0.816171f },
    { 2.362775f, 4.983935f, -5.105836f, -10.294503f, 0.027780f },
    { -0.882629f, -7.079597f, -7.051306f, 3.216183f, 13.488666f },
    { 13.465839f, 11.152921f, 26.775121f, 13.573269f, 10.633919f },
    { 9.705066f, 4.389028f, 10.082304f, 1.788778f, 7.817813f },
    { -5.251650f, 3.159873f, -7.061683f, -9.009171f, -1.071226f },
    { 3.857198f, 11.172666f, 8.647387f, -4.205257f, 8.491227f },
    { 2.142788f, 3.735756f, -4.218645f, 7.100564f, 1.246171f },
    { 5.123444f, 3.365980f, 8.465340f, -0.876098f, -2.966821f },
    { 10.216090f, -0.501517f, 14.590200f, -4.792276f, 8.162184f },
    { 6.822696f, 5.669001f, 17.653280f, 0.981210f, 7.659127f },
    { 6.028419f, 3.076205f, 2.922259f, 2.697078f, 5.408301f },
    { 8.735000f, 10.777673f, 12.286050f, 8.871432f, 13.295936f },
    { 13.269659f, 12.061034f, 15.778739f, 5.405682f, -0.377035f },
    { 9.154149f, 8.107533f, 7.067214f, -5.223533f, -4.545866f },
    { 6.550291f, -0.338672f, -1.650404f, -7.101381f, -8.842012f },
    { 8.358473f, -3.396435f, 15.449561f, 12.320868f, 12.339810f },
    { 8.903976f, -12.994958f, 15.801267f, -7.378777f, 4.248786f },
    { -2.113280f, 4.477491f, -5.060414f, -5.803987f, 6.453842f },
    { -9.486559f, -5.253564f, -13.282792f, -19.598006f, -0.391519f },
    { 3.478389f, 11.687853f, -7.459382f, 7.687440f, -1.433286f },
    { 9.501981f, 0.974160f, 14.018018f, 2.869074f, 3.722916f },
    { 8.023709f, 2.106223f, 9.482573f, -1.428144f, 2.988848f },
    { 1.786183f, -7.289167f, 0.891463f, -4.820219f, -3.988556f },
    { 4.802988f, 4.016815f, 3.621223f, -2.458844f, 5.217615f },
    { 3.342873f, -1.958178f, -3.279505f, -8.455911f, -10.916089f },
    { 2.503450f, -4.818226f, 5.521494f, 0.764726f, 8.326801f },
    { -3.000568f, -8.076841f, -4.192793f, -0.695308f, 8.670088f },
    { 6.247291f, -5.489693f, 9.131945f, 6.085129f, 4.712612f },
    { -5.632326f, -4.954839f, -25.417610f, -13.858641f, -14.100713f },
    { 2.817310f, 7.067089f, 6.235104f, -10.295631f, 1.625912f },
    { 2.147420f, -11.685836f, 6.045424f, -6.428482f, 7.726032f },
    { -0.690035f, 6.553134f, -1.353754f, 0.745319f, 11.584778f },
    { -0.102043f, 3.190333f, -1.835264f, -0.251803f, -0.378604f },
    { -7.216774f, -13.702096f, -17.598279f, -9.867622f, 3.330532f },
    { -0.839712f, -2.769702f, -2.335781f, 3.506887f, 0.934471f },
    { 5.987792f, 1.108763f, -0.464126f, 15.402795f, 9.546995f },
    { 6.695701f, -1.124529f, 14.521753f, 7.340440f, 16.606691f },
    { 9.322948f, -1.284717f, 22.001795f, 14.007745f, 8.373850f },
    { 3.721696f, 8.227501f, 1.675280f, 2.393227f, 6.627779f },
    { 1.338097f, -13.054152f, -6.473452f, -7.508878f, -1.107738f },
    { -11.677942f, -16.376653f, -8.311850f, -8.481590f, 9.632077f },
    { -4.371975f, -0.069344f, -9.061009f, 5.153092f, 1.869720f },
    { -0.906873f, 2.784427f, -7.867307f, -14.734302f, -11.340162f },
    { -0.814983f, 5.136337f, -4.702854f, -5.024645f, -2.797448f },
    { -5.080748f, -2.701587f, -12.835444f, -4.245601f, 0.024018f },
    { 2.537532f, 12.175776f, 0.066133f, -2.643332f, -5.013271f },
    { 5.159624f, 10.283226f, 2.472253f, 0.769470f, 0.746974f },
    { 3.814629f, 8.534816f, -1.087498f, 1.744127f, -0.107357f },
    { 9.495900f, 2.896898f, 15.405420f, 9.739257f, 10.538219f },
    { 8.688752f, 7.037931f, 5.163519f, 3.378810f, 5.443479f },
    { -1.004292f, 2.221216f, -4.925377f, -8.510681f, 8.079143f },
    { -4.453774f, -6.371383f, -9.887528f, -11.013895f, -2.870454f },
    { -1.256828f, -9.015011f, -2.371580f, -1.956610f, 5.509022f },
    { 5.502267f, 9.860403f, 5.205116f, -0.927544f, 8.870687f },
    { 1.389863f, 1.825889f, -8.708828f, -1.003733f, 5.017207f },
    { -10.636543f, -7.512182f, -10.000123f, -10.438254f, 0.485703f },
    { -1.086297f, -7.848708f, -1.966113f, -3.719332f, 1.283740f },
    { 7.078193f, 1.054980f, 10.794133f, 13.931773f, 9.394169f },
    { 4.151657f, 5.133885f, -3.503191f, 0.996222f, -1.613901f },
    { 3.397166f, -1.787762f, 0.335006f, -2.505524f, -1.050812f },
    { -2.163016f, -4.112018f, -10.617688f, -3.906920f, -3.556482f },
    { 5.993656f, 3.328412f, 9.928363f, 14.139877f, 9.749489f },
    { 1.199505f, -2.393211f, 0.345732f, -5.536986f, 7.642406f },
    { -3.358885f, -7.722886f, -7.065172f, -7.110231f, -8.398362f },
    { 11.630529f, 5.856937f, 21.283410f, 4.126965f, 19.194555f },
    { 10.405660f, -4.781873f, 15.375171f, 9.376211f, -0.190064f },
    { 11.958637f, 2.743373f, 20.351785f, 9.267624f, 13.271688f },
    { 12.893991f, -1.812703f, 19.890712f, 1.982468f, 13.772666f },
    { 4.635862f, 4.648977f, -6.547882f, 1.980314f, 1.296880f },
    { 9.360109f, 3.951184f, 7.335327f, 4.888144f, 1.208709f },
    { 5.511210f, 2.201839f, -0.083465f, -1.583410f, -4.779692f },
    { 2.204934f, 11.322980f, -14.837388f, -3.243380f, -7.944686f },
    { 3.354066f, -1.608929f, 16.290500f, 0.410972f, 12.855191f },
    { 7.980852f, 2.007237f, 16.499131f, 8.256451f, 10.326337f },
};

static const float32_t test_x[TEST_SAMPLE_NUM][FEATURE_DIM] = {
    { 1.000000f, 0.544915f, -0.091051f, 1.486835f, 0.677361f, -1.416987f, -1.438638f, 1.118824f, -1.508138f, 0.091406f },
    { 1.000000f, -0.325164f, -0.137112f, -0.086474f, 0.260290f, 1.504464f, -1.162580f, 1.392745f, -0.377379f, -1.402591f },
    { 1.000000f, -0.859415f, 0.479381f, 0.141531f, -0.580969f, 0.377469f, -1.035445f, 0.976670f, 0.416420f, 0.566938f },
    { 1.000000f, 0.991530f, 0.837493f, 0.213611f, 0.634109f, 0.908845f, -2.469430f, -0.103539f, -0.367553f, 1.859570f },
    { 1.000000f, -0.384222f, -0.293648f, -0.793351f, 0.585722f, -0.942045f, 0.493089f, -1.918894f, -0.612269f, -0.134458f },
    { 1.000000f, 0.214040f, -0.816572f, 1.014186f, -1.526446f, 0.811584f, -0.356877f, -0.334315f, -2.202216f, -1.193798f },
    { 1.000000f, 1.637603f, -0.379022f, -0.164221f, 0.141701f, -1.441297f, 1.348039f, -1.130697f, -0.591569f, -1.788959f },
    { 1.000000f, 0.208828f, -0.231735f, -0.840470f, -0.124013f, -1.071864f, 1.406355f, 0.282147f, 0.805530f, -0.145204f },
    { 1.000000f, 0.862469f, 0.364936f, 0.602110f, -0.019047f, 0.999538f, -1.099750f, 0.582771f, 0.693755f, 1.341880f },
    { 1.000000f, 0.399093f, 0.473211f, 1.863069f, -0.313110f, 0.800260f, -0.069576f, -0.198527f, 0.483520f, -0.681211f },
};

static const float32_t test_y[TEST_SAMPLE_NUM][TARGET_DIM] = {
    { 6.330951f, 7.818419f, 2.299738f, -1.843157f, -7.248245f },
    { -1.672581f, 2.290343f, -4.547452f, 4.338773f, 6.069390f },
    { 5.390421f, 1.744641f, -0.169690f, 4.174136f, 2.247472f },
    { 8.800734f, 8.657371f, 7.527491f, -6.898635f, -6.990787f },
    { -0.516486f, -8.157031f, -1.806340f, -12.753118f, 1.761827f },
    { -6.900781f, -1.904472f, -17.633457f, -11.268463f, -11.502399f },
    { -0.741188f, 2.057506f, 3.358061f, -3.726068f, 6.194049f },
    { 6.294264f, -0.915739f, 12.371279f, 11.784517f, 10.083503f },
    { 9.311591f, 8.878288f, 12.310731f, 7.456711f, 0.584131f },
    { 6.683621f, 5.368638f, 5.613621f, 5.159694f, 3.697751f },
};

static const float32_t train_y_vec[TRAIN_SAMPLE_NUM] = {
    8.278260f, -3.013253f, 6.688184f, 15.500402f, 3.784640f,
    9.564030f, 2.963589f, 0.393391f, 6.749493f, 5.606434f,
    8.063377f, 1.393995f, 8.884985f, 2.431600f, 1.288904f,
    -2.172336f, 1.080699f, 10.030510f, -7.191637f, 1.785179f,
    6.517975f, 7.821981f, 8.791829f, 2.048977f, 8.218707f,
    5.738932f, 2.362775f, -0.882629f, 13.465839f, 9.705066f,
    -5.251650f, 3.857198f, 2.142788f, 5.123444f, 10.216090f,
    6.822696f, 6.028419f, 8.735000f, 13.269659f, 9.154149f,
    6.550291f, 8.358473f, 8.903976f, -2.113280f, -9.486559f,
    3.478389f, 9.501981f, 8.023709f, 1.786183f, 4.802988f,
    3.342873f, 2.503450f, -3.000568f, 6.247291f, -5.632326f,
    2.817310f, 2.147420f, -0.690035f, -0.102043f, -7.216774f,
    -0.839712f, 5.987792f, 6.695701f, 9.322948f, 3.721696f,
    1.338097f, -11.677942f, -4.371975f, -0.906873f, -0.814983f,
    -5.080748f, 2.537532f, 5.159624f, 3.814629f, 9.495900f,
    8.688752f, -1.004292f, -4.453774f, -1.256828f, 5.502267f,
    1.389863f, -10.636543f, -1.086297f, 7.078193f, 4.151657f,
    3.397166f, -2.163016f, 5.993656f, 1.199505f, -3.358885f,
    11.630529f, 10.405660f, 11.958637f, 12.893991f, 4.635862f,
    9.360109f, 5.511210f, 2.204934f, 3.354066f, 7.980852f,
};

static const float32_t test_y_vec[TEST_SAMPLE_NUM] = {
    6.330951f, -1.672581f, 5.390421f, 8.800734f, -0.516486f,
    -6.900781f, -0.741188f, 6.294264f, 9.311591f, 6.683621f,
};

static const float32_t _coef1 = -1.112685f;
static const float32_t _bias1 = 2.021848f;

static const float32_t train_x1[TRAIN_SAMPLE_NUM] = {
    1.142257f, -0.630813f, 0.166629f, 0.297229f, 1.338726f,
    -0.956586f, -0.338125f, 0.430848f, 1.709946f, -1.241249f,
    -1.428380f, -0.229324f, 0.953880f, -0.777821f, 1.085221f,
    1.296405f, 1.109198f, 0.226433f, 0.448067f, -0.497561f,
    -1.350750f, 0.111726f, 2.035093f, 1.553564f, -0.607705f,
    0.082312f, -0.099591f, 1.265449f, -0.346259f, -1.419640f,
    -0.641793f, 0.406486f, -0.885065f, 0.050363f, 2.670863f,
    0.384638f, -0.971495f, -1.763498f, -0.215715f, 0.883899f,
    -0.115701f, 0.470592f, 1.058394f, -0.548491f, 0.137810f,
    -0.075925f, -1.122003f, 0.720035f, -0.872611f, 0.583195f,
    -2.758785f, -0.362698f, -1.272447f, -0.924374f, -1.972367f,
    -1.432071f, -0.911674f, -0.274904f, 1.132270f, 0.402088f,
    -1.798605f, -0.002608f, -0.908273f, -0.200794f, -1.245420f,
    0.160205f, -0.707152f, 0.773492f, 1.049714f, -0.353458f,
    1.018007f, 0.137307f, -0.445303f, 2.121736f, -1.019257f,
    -1.066884f, 0.488948f, -1.118029f, -0.386619f, 0.076391f,
    -0.062771f, 1.299745f, 0.593947f, 0.026642f, -2.026838f,
    0.425422f, 0.727174f, 0.440647f, -0.266339f, -1.353919f,
    -1.203846f, 0.251612f, -0.735398f, -1.990578f, -0.462275f,
    -1.685612f, 1.271445f, 0.207510f, 0.566973f, 0.695867f,
};

static const float32_t train_y1[TRAIN_SAMPLE_NUM] = {
    0.871180f, 2.714160f, 2.842059f, 2.205947f, 0.223724f,
    3.681623f, 2.173118f, 1.834825f, -0.520675f, 4.376253f,
    4.719494f, 0.421343f, 1.715786f, 2.054999f, 0.913606f,
    -1.167278f, 2.715400f, 1.191953f, 1.430126f, 2.281428f,
    5.316168f, 2.026190f, -1.105598f, -0.338696f, 3.658955f,
    2.512587f, 2.514431f, 1.777685f, 0.076539f, 2.873357f,
    1.393159f, 1.629299f, 3.681363f, 3.393799f, -1.154265f,
    1.977070f, 3.134039f, 4.708350f, 3.061728f, 1.667832f,
    1.197022f, 0.922096f, 1.286784f, 3.078500f, 2.674967f,
    0.943979f, 3.401221f, 0.897681f, 1.910902f, 1.794356f,
    5.746739f, 0.679006f, 6.280164f, 3.447493f, 6.513273f,
    3.827210f, 3.101827f, 3.130859f, 0.856764f, 1.836703f,
    2.786974f, 0.846295f, 1.957769f, -0.746256f, 2.879810f,
    3.173670f, 2.242648f, 2.793535f, 0.831612f, 3.293291f,
    2.132151f, 0.679998f, 2.699540f, -0.791891f, 2.818038f,
    3.851832f, 4.263393f, 3.704964f, 2.942738f, 2.910805f,
    2.846802f, 1.904144f, 0.966293f, 2.142175f, 4.618839f,
    0.871546f, -0.325098f, 2.928784f, 3.691573f, 1.520327f,
    3.563377f, 1.902009f, 3.215589f, 3.831987f, 1.584320f,
    4.355893f, 0.581808f, 0.173024f, 0.732240f, 1.881225f,
};

static const float32_t test_x1[TEST_SAMPLE_NUM] = {
    0.554387f, 0.664538f, -0.291915f, -1.899251f, 0.961059f,
    0.305794f, 0.669844f, 0.951144f, 0.011565f, 0.152611f,
};

static const float32_t test_y1[TEST_SAMPLE_NUM] = {
    2.246101f, 2.822751f, 2.588140f, 4.107105f, 0.504382f,
    1.505267f, 1.129884f, 2.757904f, -0.675186f, 1.278381f,
};

#endif  /* _DATA_H */
