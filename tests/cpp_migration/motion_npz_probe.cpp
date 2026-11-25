#include <cnpy.h>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
void check(const cnpy::NpyArray& a) {
    if(a.shape!=std::vector<size_t>{3,4} || a.word_size!=8) throw std::runtime_error("numeric shape mismatch");
    for(size_t i=0;i<12;++i) if(a.data<double>()[i]!=double(i)/8.) throw std::runtime_error("numeric value mismatch");
}
int main(int argc,char** argv) {
    try {
        if(argc!=2) throw std::runtime_error("usage: motion_npz_probe FIXTURE_DIRECTORY");
        const std::string dir=argv[1];
        for(const auto mode:{"stored","compressed"}) {
            const auto path=dir+"/"+mode+".npz";
            const auto all=cnpy::npz_load(path);
            if(all.size()!=3) throw std::runtime_error("member count mismatch");
            check(all.at("matrix"));check(cnpy::npz_load(path,"matrix"));
            const auto names=cnpy::npz_load(path,"names");
            if(names.shape!=std::vector<size_t>{2} || names.word_size!=20 || names.data<uint32_t>()[0]!=uint32_t('a') || names.data<uint32_t>()[5]!=0x7ad9)
                throw std::runtime_error("Unicode code points mismatch");
            const auto scalar=cnpy::npz_load(path,"scalar");
            if(!scalar.shape.empty() || scalar.word_size!=8 || scalar.data<double>()[0]!=3.25) throw std::runtime_error("scalar mismatch");
        }
        check(cnpy::npy_load(dir+"/v2.npy"));
        std::cout<<"NPZ stored/compressed ZIP64, Unicode, scalar, member selection and NPY v2 passed\n";
        return 0;
    } catch(const std::exception& error) { std::cerr<<error.what()<<"\n";return 1; }
}
