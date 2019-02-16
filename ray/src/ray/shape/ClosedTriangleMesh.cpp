#include "ClosedTriangleMesh.h"

namespace ray
{
    ClosedTriangleMeshFace::ClosedTriangleMeshFace(const ClosedTriangleMesh& mesh, int i0, int i1, int i2) :
        m_vertices{ &mesh.vertex(i0), &mesh.vertex(i1), &mesh.vertex(i2) }
    {
    }
}