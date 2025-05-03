#include "BuildFileDescriptorSet.hpp"

#include <queue>
#include <string>
#include <unordered_set>

namespace foxglove {

/// Builds a FileDescriptorSet of this descriptor and all transitive
/// dependencies, for use as a channel schema.
google::protobuf::FileDescriptorSet BuildFileDescriptorSet(
    google::protobuf::Descriptor const* toplevelDescriptor)
{
    google::protobuf::FileDescriptorSet fdSet;
    std::queue<google::protobuf::FileDescriptor const*> toAdd;
    toAdd.push(toplevelDescriptor->file());
    std::unordered_set<std::string> seenDependencies;
    while (!toAdd.empty()) {
        google::protobuf::FileDescriptor const* next = toAdd.front();
        toAdd.pop();
        next->CopyTo(fdSet.add_file());
        for (int i = 0; i < next->dependency_count(); ++i) {
            auto const& dep = next->dependency(i);
            if (seenDependencies.find(dep->name()) == seenDependencies.end()) {
                seenDependencies.insert(dep->name());
                toAdd.push(dep);
            }
        }
    }
    return fdSet;
}

} // namespace foxglove
