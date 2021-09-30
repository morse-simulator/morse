# Unique file ID
@0x940bb1aef1e15f74;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("enc");

struct Object {
    objName @0 :Text;
    kd @1 :Float32;
    ks @2 :Float32;
    scaleX @3 :Float32;
    scaleY @4 :Float32;
    scaleZ @5 :Float32;
    rho @6 :Float32;
    speed @7 :Float32;
    alpha @8 :Float32;
    dataName @9 :Text;
    dataBlock @10 :DataBlock;
}
    
struct DataBlock {
    meshColour @0 :List(Float32);   # 3 x float
    dims @1 :List(Int32);           # rows, cols
    texture @2 :List(Float32);      # vector<float> flattened from vector<RGBA>
    uvs @3 :List(Float32);		    # vector<float> flattened from vector<2xfloat>
    mesh @4 :Mesh;
}

struct Mesh {
    vertices @0 :List(Float32); # vector<float> flattened from vector<point> where point=(3xfloat)
    faces @1 :List(Int32);		# vector<face vertex id>
}
