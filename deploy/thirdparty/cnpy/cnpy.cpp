//Copyright (C) 2011  Carl Rogers
//Released under MIT License
//license available in LICENSE file, or at http://www.opensource.org/licenses/mit-license.php

#include"cnpy.h"
#include<complex>
#include<cstdlib>
#include<algorithm>
#include<cstring>
#include<iomanip>
#include<stdint.h>
#include<stdexcept>
#include <regex>

char cnpy::BigEndianTest() {
    int x = 1;
    return (((char *)&x)[0]) ? '<' : '>';
}

char cnpy::map_type(const std::type_info& t)
{
    if(t == typeid(float) ) return 'f';
    if(t == typeid(double) ) return 'f';
    if(t == typeid(long double) ) return 'f';

    if(t == typeid(int) ) return 'i';
    if(t == typeid(char) ) return 'i';
    if(t == typeid(short) ) return 'i';
    if(t == typeid(long) ) return 'i';
    if(t == typeid(long long) ) return 'i';

    if(t == typeid(unsigned char) ) return 'u';
    if(t == typeid(unsigned short) ) return 'u';
    if(t == typeid(unsigned long) ) return 'u';
    if(t == typeid(unsigned long long) ) return 'u';
    if(t == typeid(unsigned int) ) return 'u';

    if(t == typeid(bool) ) return 'b';

    if(t == typeid(std::complex<float>) ) return 'c';
    if(t == typeid(std::complex<double>) ) return 'c';
    if(t == typeid(std::complex<long double>) ) return 'c';

    else return '?';
}

template<> std::vector<char>& cnpy::operator+=(std::vector<char>& lhs, const std::string rhs) {
    lhs.insert(lhs.end(),rhs.begin(),rhs.end());
    return lhs;
}

template<> std::vector<char>& cnpy::operator+=(std::vector<char>& lhs, const char* rhs) {
    //write in little endian
    size_t len = strlen(rhs);
    lhs.reserve(len);
    for(size_t byte = 0; byte < len; byte++) {
        lhs.push_back(rhs[byte]);
    }
    return lhs;
}

namespace {
uint16_t le16(const unsigned char* p) { return uint16_t(p[0]) | (uint16_t(p[1])<<8); }
uint32_t le32(const unsigned char* p) { return uint32_t(le16(p)) | (uint32_t(le16(p+2))<<16); }
uint64_t le64(const unsigned char* p) { return uint64_t(le32(p)) | (uint64_t(le32(p+4))<<32); }
void read_exact(FILE* fp,void* dst,size_t bytes) {
    if(fread(dst,1,bytes,fp)!=bytes) throw std::runtime_error("cnpy: truncated file");
}
void parse_header_text(const std::string& header,size_t& word_size,std::vector<size_t>& shape,bool& fortran_order) {
    std::smatch match;
    if(!std::regex_search(header,match,std::regex("'fortran_order'\\s*:\\s*(True|False)")))
        throw std::runtime_error("cnpy: invalid fortran_order");
    fortran_order=match[1]=="True";
    if(!std::regex_search(header,match,std::regex("'shape'\\s*:\\s*\\(([^)]*)\\)")))
        throw std::runtime_error("cnpy: invalid shape");
    std::string dims=match[1];shape.clear();
    const std::regex digits("[0-9]+");
    while(std::regex_search(dims,match,digits)) { shape.push_back(std::stoull(match[0]));dims=match.suffix(); }
    if(!std::regex_search(header,match,std::regex("'descr'\\s*:\\s*'([<|=])([A-Za-z?])([0-9]+)'")))
        throw std::runtime_error("cnpy: only little-endian scalar NumPy dtypes are supported");
    word_size=std::stoull(match[3]);
    // NumPy Unicode's itemsize is four bytes per UTF-32 code point.
    if(match[2]=="U") word_size*=4;
    if(!word_size) throw std::runtime_error("cnpy: invalid itemsize");
}
size_t npy_prefix(const unsigned char* p) {
    if(std::memcmp(p,"\x93NUMPY",6)!=0 || (p[6]!=1 && p[6]!=2 && p[6]!=3))
        throw std::runtime_error("cnpy: unsupported NumPy header");
    return p[6]==1 ? 10 : 12;
}
struct ZipMember { std::string name;uint16_t method;uint32_t compressed,uncompressed; };
bool read_member(FILE* fp,ZipMember& out) {
    unsigned char h[30];read_exact(fp,h,sizeof(h));
    const uint32_t magic=le32(h);
    if(magic==0x02014b50 || magic==0x06054b50 || magic==0x06064b50) return false;
    if(magic!=0x04034b50) throw std::runtime_error("cnpy: invalid ZIP local header");
    if(le16(h+6)&9) throw std::runtime_error("cnpy: encrypted/streaming ZIP members unsupported");
    const size_t name_size=le16(h+26),extra_size=le16(h+28);
    out.name.resize(name_size);read_exact(fp,&out.name[0],name_size);
    if(out.name.size()<4 || out.name.substr(out.name.size()-4)!=".npy") throw std::runtime_error("cnpy: non-NPY ZIP member");
    out.name.resize(out.name.size()-4);
    std::vector<unsigned char> extra(extra_size);read_exact(fp,extra.data(),extra.size());
    uint64_t compressed=le32(h+18),uncompressed=le32(h+22);
    const bool big_c=compressed==0xffffffffu,big_u=uncompressed==0xffffffffu;
    bool zip64=false;
    for(size_t offset=0;offset+4<=extra.size();) {
        const size_t length=le16(extra.data()+offset+2),start=offset+4;
        if(start+length>extra.size()) throw std::runtime_error("cnpy: truncated ZIP extra field");
        if(le16(extra.data()+offset)==1) {
            size_t pos=start;
            if(big_u) { if(pos+8>start+length) throw std::runtime_error("cnpy: missing ZIP64 length");uncompressed=le64(extra.data()+pos);pos+=8; }
            if(big_c) { if(pos+8>start+length) throw std::runtime_error("cnpy: missing ZIP64 length");compressed=le64(extra.data()+pos); }
            zip64=true;
        }
        offset=start+length;
    }
    if((big_c || big_u) && !zip64) throw std::runtime_error("cnpy: missing ZIP64 extra field");
    // zlib's single-buffer API uses 32-bit lengths; fail clearly above its limit.
    if(compressed>0xffffffffu || uncompressed>0xffffffffu) throw std::runtime_error("cnpy: ZIP member exceeds 4 GiB");
    out.compressed=uint32_t(compressed);out.uncompressed=uint32_t(uncompressed);out.method=le16(h+8);
    if(out.method!=0 && out.method!=8) throw std::runtime_error("cnpy: unsupported ZIP compression");
    return true;
}
}
void cnpy::parse_npy_header(unsigned char* buffer,size_t& word_size,std::vector<size_t>& shape,bool& fortran_order) {
    const auto prefix=npy_prefix(buffer);
    const size_t length=prefix==10 ? le16(buffer+8) : le32(buffer+8);
    parse_header_text(std::string(reinterpret_cast<char*>(buffer+prefix),length),word_size,shape,fortran_order);
}
void cnpy::parse_npy_header(FILE* fp,size_t& word_size,std::vector<size_t>& shape,bool& fortran_order) {
    unsigned char pre[12]={};read_exact(fp,pre,8);
    const auto prefix=npy_prefix(pre);read_exact(fp,pre+8,prefix-8);
    const size_t length=prefix==10 ? le16(pre+8) : le32(pre+8);
    if(length==0 || length>1048576) throw std::runtime_error("cnpy: invalid NumPy header length");
    std::string header(length,' ');read_exact(fp,&header[0],header.size());
    parse_header_text(header,word_size,shape,fortran_order);
}

void cnpy::parse_zip_footer(FILE* fp, uint16_t& nrecs, size_t& global_header_size, size_t& global_header_offset)
{
    std::vector<char> footer(22);
    fseek(fp,-22,SEEK_END);
    size_t res = fread(&footer[0],sizeof(char),22,fp);
    if(res != 22)
        throw std::runtime_error("parse_zip_footer: failed fread");

    uint16_t disk_no, disk_start, nrecs_on_disk, comment_len;
    disk_no = *(uint16_t*) &footer[4];
    disk_start = *(uint16_t*) &footer[6];
    nrecs_on_disk = *(uint16_t*) &footer[8];
    nrecs = *(uint16_t*) &footer[10];
    global_header_size = *(uint32_t*) &footer[12];
    global_header_offset = *(uint32_t*) &footer[16];
    comment_len = *(uint16_t*) &footer[20];

    assert(disk_no == 0);
    assert(disk_start == 0);
    assert(nrecs_on_disk == nrecs);
    assert(comment_len == 0);
}

cnpy::NpyArray load_the_npy_file(FILE* fp) {
    std::vector<size_t> shape;
    size_t word_size;
    bool fortran_order;
    cnpy::parse_npy_header(fp,word_size,shape,fortran_order);

    cnpy::NpyArray arr(shape, word_size, fortran_order);
    size_t nread = fread(arr.data<char>(),1,arr.num_bytes(),fp);
    if(nread != arr.num_bytes())
        throw std::runtime_error("load_the_npy_file: failed fread");
    return arr;
}

cnpy::NpyArray load_the_npz_array(FILE* fp, uint32_t compr_bytes, uint32_t uncompr_bytes) {

    std::vector<unsigned char> buffer_compr(compr_bytes);
    std::vector<unsigned char> buffer_uncompr(uncompr_bytes);
    size_t nread = fread(&buffer_compr[0],1,compr_bytes,fp);
    if(nread != compr_bytes)
        throw std::runtime_error("load_the_npy_file: failed fread");

    int err;
    z_stream d_stream;

    d_stream.zalloc = Z_NULL;
    d_stream.zfree = Z_NULL;
    d_stream.opaque = Z_NULL;
    d_stream.avail_in = 0;
    d_stream.next_in = Z_NULL;
    err = inflateInit2(&d_stream, -MAX_WBITS);
    if(err != Z_OK) throw std::runtime_error("cnpy: inflateInit2 failed");

    d_stream.avail_in = compr_bytes;
    d_stream.next_in = &buffer_compr[0];
    d_stream.avail_out = uncompr_bytes;
    d_stream.next_out = &buffer_uncompr[0];

    err = inflate(&d_stream, Z_FINISH);
    const bool complete = err == Z_STREAM_END && d_stream.total_out == uncompr_bytes;
    inflateEnd(&d_stream);
    if(!complete) throw std::runtime_error("cnpy: invalid compressed NumPy member");

    if(buffer_uncompr.size()<12) throw std::runtime_error("cnpy: truncated NumPy member");
    const size_t header_prefix=npy_prefix(buffer_uncompr.data());
    const size_t header_size=header_prefix==10 ? le16(buffer_uncompr.data()+8) : le32(buffer_uncompr.data()+8);
    if(header_size>buffer_uncompr.size()-header_prefix) throw std::runtime_error("cnpy: truncated NumPy header");

    std::vector<size_t> shape;
    size_t word_size;
    bool fortran_order;
    cnpy::parse_npy_header(&buffer_uncompr[0],word_size,shape,fortran_order);

    cnpy::NpyArray array(shape, word_size, fortran_order);

    const size_t prefix=npy_prefix(buffer_uncompr.data());
    const size_t offset=prefix+(prefix==10 ? le16(buffer_uncompr.data()+8) : le32(buffer_uncompr.data()+8));
    if(offset>uncompr_bytes || array.num_bytes()!=uncompr_bytes-offset)
        throw std::runtime_error("cnpy: NumPy data size mismatch");
    memcpy(array.data<unsigned char>(),&buffer_uncompr[0]+offset,array.num_bytes());

    return array;
}

cnpy::npz_t cnpy::npz_load(std::string fname) {
    std::unique_ptr<FILE,decltype(&fclose)> fp(fopen(fname.c_str(),"rb"),&fclose);
    if(!fp) throw std::runtime_error("npz_load: unable to open "+fname);
    npz_t arrays;ZipMember member;
    while(read_member(fp.get(),member)) {
        arrays[member.name]=member.method==0 ? load_the_npy_file(fp.get()) :
            load_the_npz_array(fp.get(),member.compressed,member.uncompressed);
    }
    return arrays;
}
cnpy::NpyArray cnpy::npz_load(std::string fname,std::string varname) {
    std::unique_ptr<FILE,decltype(&fclose)> fp(fopen(fname.c_str(),"rb"),&fclose);
    if(!fp) throw std::runtime_error("npz_load: unable to open "+fname);
    ZipMember member;
    while(read_member(fp.get(),member)) {
        if(member.name==varname) return member.method==0 ? load_the_npy_file(fp.get()) :
            load_the_npz_array(fp.get(),member.compressed,member.uncompressed);
        // ZIP offsets always use the compressed length, including ZIP64 archives.
        if(fseek(fp.get(),member.compressed,SEEK_CUR)!=0) throw std::runtime_error("npz_load: seek failed");
    }
    throw std::runtime_error("npz_load: variable "+varname+" not found in "+fname);
}

cnpy::NpyArray cnpy::npy_load(std::string fname) {

    FILE* fp = fopen(fname.c_str(), "rb");

    if(!fp) throw std::runtime_error("npy_load: Unable to open file "+fname);

    NpyArray arr = load_the_npy_file(fp);

    fclose(fp);
    return arr;
}



