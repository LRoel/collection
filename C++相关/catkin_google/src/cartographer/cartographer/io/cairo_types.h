#ifndef CARTOGRAPHER_IO_CAIRO_TYPES_H_
#define CARTOGRAPHER_IO_CAIRO_TYPES_H_

#include <memory>

#include "cairo/cairo.h"

namespace cartographer {
namespace io {
namespace cairo {

// std::unique_ptr for Cairo surfaces. The surface is destroyed when the
// std::unique_ptr is reset or destroyed.
/**
* @brief std :: unique_ptr用于开罗表面.当std :: unique_ptr重置或销毁时，表面将被破坏.
*/
using UniqueSurfacePtr =
    std::unique_ptr<cairo_surface_t, void (*)(cairo_surface_t*)>;

// std::unique_ptr for Cairo contexts. The context is destroyed when the
// std::unique_ptr is reset or destroyed.
/**
* @brief std :: unique_ptr用于开罗上下文.当std :: unique_ptr被重置或销毁时，上下文被破坏.
*/
using UniqueContextPtr = std::unique_ptr<cairo_t, void (*)(cairo_t*)>;

// std::unique_ptr for Cairo paths. The path is destroyed when the
// std::unique_ptr is reset or destroyed.
/**
* @brief std :: unique_ptr用于开罗路径.当std :: unique_ptr被重置或销毁时，路径被破坏.
*/
using UniquePathPtr = std::unique_ptr<cairo_path_t, void (*)(cairo_path_t*)>;

}  // namespace cairo
}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_CAIRO_TYPES_H_
