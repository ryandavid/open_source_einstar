#include "dialogs.hpp"

#import <Cocoa/Cocoa.h>
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

namespace einstar::modelapp {
namespace {

NSArray<UTType*>* types_of(const std::vector<std::string>& extensions) {
    NSMutableArray<UTType*>* types = [NSMutableArray array];
    for (const auto& e : extensions) {
        UTType* t = [UTType typeWithFilenameExtension:[NSString stringWithUTF8String:e.c_str()]];
        if (t) [types addObject:t];
    }
    return types;
}

}  // namespace

std::optional<std::filesystem::path> choose_open_file(const std::vector<std::string>& extensions) {
    NSOpenPanel* panel = [NSOpenPanel openPanel];
    panel.canChooseFiles = YES;
    panel.canChooseDirectories = NO;
    panel.allowsMultipleSelection = NO;
    panel.allowedContentTypes = types_of(extensions);
    if ([panel runModal] != NSModalResponseOK || panel.URLs.count == 0) return std::nullopt;
    return std::filesystem::path(panel.URLs.firstObject.fileSystemRepresentation);
}

std::optional<std::filesystem::path> choose_save_file(const std::string& name, const std::string& extension) {
    NSSavePanel* panel = [NSSavePanel savePanel];
    panel.nameFieldStringValue = [NSString stringWithUTF8String:name.c_str()];
    panel.allowedContentTypes = types_of({extension});
    if ([panel runModal] != NSModalResponseOK || !panel.URL) return std::nullopt;
    return std::filesystem::path(panel.URL.fileSystemRepresentation);
}

}  // namespace einstar::modelapp
