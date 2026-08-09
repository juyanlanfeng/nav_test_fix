#include <rviz_mesh_tools_plugins/InteractionHelper.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/render_window.hpp>

#include <OgreViewport.h>
#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreTechnique.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace rviz_mesh_tools_plugins
{

Ogre::Ray getMouseEventRay(
  const rviz_common::ViewportMouseEvent& event)
{
  auto viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow());
  int width = viewport->getActualWidth();
  int height = viewport->getActualHeight();
  Ogre::Ray mouse_ray = viewport->getCamera()->getCameraToViewportRay(
    static_cast<float>(event.x) / static_cast<float>(width),
    static_cast<float>(event.y) / static_cast<float>(height));
  return mouse_ray;
}


void getRawManualObjectData(
  const Ogre::Affine3& to_world,
  Ogre::ManualObject::ManualObjectSection& section,
  size_t& vertexCount, Ogre::Vector3*& vertices, 
  size_t& indexCount, unsigned long*& indices)
{
  Ogre::VertexData* vertexData;
  const Ogre::VertexElement* vertexElement;
  Ogre::HardwareVertexBufferSharedPtr vertexBuffer;
  unsigned char* vertexChar;
  float* vertexFloat;

  vertexData = section.getRenderOperation()->vertexData;
  vertexElement = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
  vertexBuffer = vertexData->vertexBufferBinding->getBuffer(vertexElement->getSource());
  vertexChar = static_cast<unsigned char*>(vertexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

  vertexCount = vertexData->vertexCount;
  vertices = new Ogre::Vector3[vertexCount];

  for (size_t i = 0; i < vertexCount; i++, vertexChar += vertexBuffer->getVertexSize())
  {
    vertexElement->baseVertexPointerToElement(vertexChar, &vertexFloat);
    vertices[i] = to_world * Ogre::Vector3(vertexFloat[0], vertexFloat[1], vertexFloat[2]);
  }

  vertexBuffer->unlock();

  Ogre::IndexData* indexData;
  Ogre::HardwareIndexBufferSharedPtr indexBuffer;
  indexData = section.getRenderOperation()->indexData;
  indexCount = indexData->indexCount;
  indices = new unsigned long[indexCount];
  indexBuffer = indexData->indexBuffer;
  unsigned int* pLong = static_cast<unsigned int*>(indexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
  unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);

  for (size_t i = 0; i < indexCount; i++)
  {
    unsigned long index;
    if (indexBuffer->getType() == Ogre::HardwareIndexBuffer::IT_32BIT)
    {
      index = static_cast<unsigned long>(pLong[i]);
    }
    else
    {
      index = static_cast<unsigned long>(pShort[i]);
    }

    indices[i] = index;
  }
  indexBuffer->unlock();
}

bool selectFace(
  const Ogre::ManualObject* mesh, 
  const Ogre::Ray& ray,
  Intersection& intersection,
  std::size_t intersection_layer)
{
  struct FaceHit
  {
    Ogre::Real distance;
    Ogre::Vector3 a;
    Ogre::Vector3 b;
    Ogre::Vector3 c;
    uint32_t face_id;
  };
  std::vector<FaceHit> hits;

  size_t vertex_count = 0;
  Ogre::Vector3* vertices;
  size_t index_count = 0;
  unsigned long* indices;
  Ogre::Affine3 mesh_to_world = mesh->_getParentNodeFullTransform();

/* ManualObject::getNumSections and ManualObject::getSection are deprecated
 * in the Ogre version ROS Jazzy (and future versions) uses. Since the new
 * API is not available in the Ogre version used by ROS Humble we use this
 * check to keep Humble support.
 *
 * This can be removed when Humble support is dropped.
 */
#if OGRE_VERSION < ((1 << 16) | (12 << 8) | 7)
  for (size_t i = 0; i < mesh->getNumSections(); i++)
  {
    const auto& section = mesh->getSection(i);
#else
  for (const auto& section: mesh->getSections())
  {
#endif
    getRawManualObjectData(mesh_to_world, *section, vertex_count, vertices, index_count, indices);

    // All sections and materials should have the same culling mode, so we use the first we find
    std::optional<Ogre::CullingMode> culling_mode;
    for (auto technique: section->getMaterial()->getTechniques())
    {
      for (auto pass: technique->getPasses())
      {
        culling_mode = pass->getCullingMode();
        break;
      }
      if (culling_mode)
      {
        break;
      }
    }

    if (!culling_mode)
    {
      culling_mode = Ogre::CullingMode::CULL_NONE;
    }

    bool int_pos = true;
    bool int_neg = true;
    switch(culling_mode.value())
    {
      case Ogre::CULL_CLOCKWISE:
        int_pos = true;
        int_neg = false;
        break;
      case Ogre::CULL_ANTICLOCKWISE:
        int_pos = false;
        int_neg = true;
        break;
      case Ogre::CULL_NONE:
        int_pos = true;
        int_neg = true;
    }

    if (index_count != 0)
    {
      for (size_t face_id = 0; face_id < index_count / 3; face_id++)
      {
        // face: indices[j], 
        const Ogre::Vector3 vertex;

        std::pair<bool, Ogre::Real> goal = Ogre::Math::intersects(ray, 
            vertices[indices[face_id * 3 + 0]], 
            vertices[indices[face_id * 3 + 1]],
            vertices[indices[face_id * 3 + 2]], int_pos, int_neg);

        if (goal.first)
        {
          hits.push_back(
            FaceHit{
              goal.second,
              vertices[indices[face_id * 3 + 0]],
              vertices[indices[face_id * 3 + 1]],
              vertices[indices[face_id * 3 + 2]],
              static_cast<uint32_t>(face_id)});
        }
      }
    }

    delete[] vertices;
    delete[] indices;
  }

  if (hits.empty()) {
    return false;
  }

  std::sort(
    hits.begin(), hits.end(),
    [](const FaceHit & left, const FaceHit & right) {
      return left.distance < right.distance;
    });

  // Adjacent triangles normally report the same physical hit.  Count them as
  // one layer, while keeping genuinely separated tunnel/roof intersections.
  constexpr Ogre::Real layer_tolerance = 1.0e-4f;
  std::size_t distinct_layer = 0;
  const FaceHit * selected = &hits.front();
  Ogre::Real previous_distance = hits.front().distance;
  if (intersection_layer > 0) {
    selected = nullptr;
    for (std::size_t index = 1; index < hits.size(); ++index) {
      if (std::abs(hits[index].distance - previous_distance) <= layer_tolerance) {
        continue;
      }
      previous_distance = hits[index].distance;
      ++distinct_layer;
      if (distinct_layer == intersection_layer) {
        selected = &hits[index];
        break;
      }
    }
  }

  if (selected)
  {
    intersection.range = selected->distance;
    intersection.point = ray.getPoint(selected->distance);
    intersection.face_id = selected->face_id;
    Ogre::Vector3 ab = selected->b - selected->a;
    Ogre::Vector3 ac = selected->c - selected->a;
    intersection.normal = ac.crossProduct(ab).normalisedCopy();
    return true;
  }
  else
  {
    return false;
  }
}

bool selectFace(
  Ogre::SceneManager* sm,
  const Ogre::Ray& ray, 
  Intersection& intersection,
  std::size_t intersection_layer)
{
  Ogre::RaySceneQuery* query = sm->createRayQuery(ray, Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
  query->setSortByDistance(true);

  Ogre::RaySceneQueryResult& result = query->execute();

  for (size_t i = 0; i < result.size(); i++)
  {
    if (result[i].movable->getName().find("TriangleMesh") != std::string::npos)
    {
      Ogre::ManualObject* mesh = static_cast<Ogre::ManualObject*>(result[i].movable);
      if (selectFace(mesh, ray, intersection, intersection_layer))
      {
        return true;
      }
    }
  }
  return false;
}

bool selectFace(
  const rviz_common::DisplayContext* ctx,
  const Ogre::Ray& ray, 
  Intersection& intersection,
  std::size_t intersection_layer)
{
  return selectFace(ctx->getSceneManager(), ray, intersection, intersection_layer);
}

} // namespace rviz_mesh_tools_plugins
