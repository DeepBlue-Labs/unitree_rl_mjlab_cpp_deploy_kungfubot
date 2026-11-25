#pragma once
#include "control/Policy.h"
#include <map>

namespace g1control {
// Registry operations never open DDS. resolve() returns a detached copy with
// defaults merged, so session/CLI overrides cannot mutate another policy.
YAML::Node readRegistry(const std::filesystem::path& path);
std::string formatPolicyList(const YAML::Node& data);
std::string fileSha256(const std::filesystem::path& path);

class Registry {
public:
    explicit Registry(std::filesystem::path root,std::filesystem::path path={});
    const YAML::Node& data() const { return data_; }
    const std::filesystem::path& root() const { return root_; }
    YAML::Node resolve(const std::string& name) const;
    std::string formatPolicyList() const;
    std::vector<std::filesystem::path> artifactPaths(const YAML::Node& spec) const;
    YAML::Node identity(const YAML::Node& spec) const;
    std::unique_ptr<Policy> load(const std::string& name) const;
    std::unique_ptr<Policy> load(const YAML::Node& spec) const;
    // Includes the resolved spec, actor contract and SHA256 identities; a ready
    // session validates all candidate policies and records their identities too.
    YAML::Node inspect(const std::string& name) const;
private:
    std::filesystem::path root_;
    YAML::Node data_;
};
}
