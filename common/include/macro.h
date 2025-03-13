#ifndef COMMON_MACRO_H
#define COMMON_MACRO_H

namespace knowledge_base{
namespace common{

#define PTR_REGISTER(name, type) \
    typedef std::shared_ptr<type>() name##Ptr; \
    typedef std::shared_ptr<const type>() name##ConstPtr; \
    typedef std::unique_ptr<type>() name##UniquePtr; \
    typedef std::unique_ptr<const type>() name##ConstUniquePtr

#define CLASS_PTR_FORWARD(C) \
    class C; \
    PTR_REGISTER(C, C)

#define STRUCT_PTR_FORWARD(S) \
    struct S; \
    PTR_REGISTER(S)

}
}

#endif