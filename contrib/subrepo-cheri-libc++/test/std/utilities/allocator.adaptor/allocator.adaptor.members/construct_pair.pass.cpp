//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

// UNSUPPORTED: c++03, libcpp-no-rtti

// <scoped_allocator>

// template <class OtherAlloc, class ...InnerAlloc>
//   class scoped_allocator_adaptor

// template <class U1, class U2>
// void scoped_allocator_adaptor::construct(pair<U1, U2>*)

#include <scoped_allocator>
#include <type_traits>
#include <utility>
#include <tuple>
#include <cassert>
#include <cstdlib>
#include "uses_alloc_types.h"
#include "controlled_allocators.h"

#include "test_macros.h"


void test_no_inner_alloc()
{
    using VoidAlloc = CountingAllocator<void>;
    AllocController P;
    {
        using T = UsesAllocatorV1<VoidAlloc, 0>;
        using U = UsesAllocatorV2<VoidAlloc, 0>;
        using Pair = std::pair<T, U>;
        using Alloc = CountingAllocator<Pair>;
        using SA = std::scoped_allocator_adaptor<Alloc>;
        static_assert(std::uses_allocator<T, CountingAllocator<T> >::value, "");
        Pair * ptr = (Pair*)std::malloc(sizeof(Pair));
        assert(ptr);
        Alloc CA(P);
        SA A(CA);
        A.construct(ptr);
        assert(checkConstruct<>(ptr->first, UA_AllocArg, CA));
        assert(checkConstruct<>(ptr->second, UA_AllocLast, CA));
#if TEST_STD_VER >= 20
        assert((P.checkConstruct<std::piecewise_construct_t&&,
                                 std::tuple<std::allocator_arg_t, const SA&>&&,
                                 std::tuple<const SA&>&&
              >(CA, ptr)));
#else
        assert((P.checkConstruct<std::piecewise_construct_t const&,
                                 std::tuple<std::allocator_arg_t, SA&>&&,
                                 std::tuple<SA&>&&
              >(CA, ptr)));
#endif
        A.destroy(ptr);
        std::free(ptr);

    }
    P.reset();
    {
        using T = UsesAllocatorV3<VoidAlloc, 0>;
        using U = NotUsesAllocator<VoidAlloc, 0>;
        using Pair = std::pair<T, U>;
        using Alloc = CountingAllocator<Pair>;
        using SA = std::scoped_allocator_adaptor<Alloc>;
        static_assert(std::uses_allocator<T, CountingAllocator<T> >::value, "");
        Pair * ptr = (Pair*)std::malloc(sizeof(Pair));
        assert(ptr);
        Alloc CA(P);
        SA A(CA);
        A.construct(ptr);
        assert(checkConstruct<>(ptr->first, UA_AllocArg, CA));
        assert(checkConstruct<>(ptr->second, UA_None));
#if TEST_STD_VER >= 20
        assert((P.checkConstruct<std::piecewise_construct_t&&,
                                 std::tuple<std::allocator_arg_t, const SA&>&&,
                                 std::tuple<>&&
                   >(CA, ptr)));
#else
        assert((P.checkConstruct<std::piecewise_construct_t const&,
                                 std::tuple<std::allocator_arg_t, SA&>&&,
                                 std::tuple<>&&
                   >(CA, ptr)));
#endif
        A.destroy(ptr);
        std::free(ptr);
    }
}

void test_with_inner_alloc()
{
    using VoidAlloc2 = CountingAllocator<void, 2>;

    AllocController POuter;
    AllocController PInner;
    {
        using T = UsesAllocatorV1<VoidAlloc2, 0>;
        using U = UsesAllocatorV2<VoidAlloc2, 0>;
        using Pair = std::pair<T, U>;
        using Outer = CountingAllocator<Pair, 1>;
        using Inner = CountingAllocator<Pair, 2>;
        using SA = std::scoped_allocator_adaptor<Outer, Inner>;
        using SAInner = std::scoped_allocator_adaptor<Inner>;
        static_assert(!std::uses_allocator<T, Outer>::value, "");
        static_assert(std::uses_allocator<T, Inner>::value, "");
        Pair * ptr = (Pair*)std::malloc(sizeof(Pair));
        assert(ptr);
        Outer O(POuter);
        Inner I(PInner);
        SA A(O, I);
        A.construct(ptr);
        assert(checkConstruct<>(ptr->first, UA_AllocArg, I));
        assert(checkConstruct<>(ptr->second, UA_AllocLast));
#if TEST_STD_VER >= 20
        assert((POuter.checkConstruct<std::piecewise_construct_t&&,
                                 std::tuple<std::allocator_arg_t, const SAInner&>&&,
                                 std::tuple<const SAInner&>&&
              >(O, ptr)));
#else
        assert((POuter.checkConstruct<std::piecewise_construct_t const&,
                                 std::tuple<std::allocator_arg_t, SAInner&>&&,
                                 std::tuple<SAInner&>&&
              >(O, ptr)));
#endif
        A.destroy(ptr);
        std::free(ptr);
    }
    PInner.reset();
    POuter.reset();
    {
        using T = UsesAllocatorV3<VoidAlloc2, 0>;
        using U = NotUsesAllocator<VoidAlloc2, 0>;
        using Pair = std::pair<T, U>;
        using Outer = CountingAllocator<Pair, 1>;
        using Inner = CountingAllocator<Pair, 2>;
        using SA = std::scoped_allocator_adaptor<Outer, Inner>;
        using SAInner = std::scoped_allocator_adaptor<Inner>;
        static_assert(!std::uses_allocator<T, Outer>::value, "");
        static_assert(std::uses_allocator<T, Inner>::value, "");
        Pair * ptr = (Pair*)std::malloc(sizeof(Pair));
        assert(ptr);
        Outer O(POuter);
        Inner I(PInner);
        SA A(O, I);
        A.construct(ptr);
        assert(checkConstruct<>(ptr->first, UA_AllocArg, I));
        assert(checkConstruct<>(ptr->second, UA_None));
#if TEST_STD_VER >= 20
        assert((POuter.checkConstruct<std::piecewise_construct_t&&,
                                 std::tuple<std::allocator_arg_t, const SAInner&>&&,
                                 std::tuple<>&&
              >(O, ptr)));
#else
        assert((POuter.checkConstruct<std::piecewise_construct_t const&,
                                 std::tuple<std::allocator_arg_t, SAInner&>&&,
                                 std::tuple<>&&
              >(O, ptr)));
#endif
        A.destroy(ptr);
        std::free(ptr);
    }
}
int main(int, char**) {
    test_no_inner_alloc();
    test_with_inner_alloc();

  return 0;
}
