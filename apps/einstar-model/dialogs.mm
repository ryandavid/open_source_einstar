#include "dialogs.hpp"

#import <Cocoa/Cocoa.h>

#include <algorithm>
#include <cctype>
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
    [NSApp activateIgnoringOtherApps:YES];  // background-launched: bring the panel to the front, keyed (see below)
    if ([panel runModal] != NSModalResponseOK || panel.URLs.count == 0) return std::nullopt;
    return std::filesystem::path(panel.URLs.firstObject.fileSystemRepresentation);
}

std::vector<std::filesystem::path> choose_open_files(const std::vector<std::string>& extensions) {
    NSOpenPanel* panel = [NSOpenPanel openPanel];
    panel.canChooseFiles = YES;
    panel.canChooseDirectories = NO;
    panel.allowsMultipleSelection = YES;
    panel.allowedContentTypes = types_of(extensions);
    std::vector<std::filesystem::path> out;
    [NSApp activateIgnoringOtherApps:YES];  // background-launched: bring the panel to the front, keyed
    if ([panel runModal] != NSModalResponseOK) return out;
    for (NSURL* url in panel.URLs) out.emplace_back(url.fileSystemRepresentation);
    return out;
}

const std::vector<std::string>& photo_extensions() {
    static const std::vector<std::string> e = {"jpg", "jpeg", "heic", "heif", "png", "tif", "tiff", "webp"};
    return e;
}

Pasted clipboard_images() {
    Pasted out;
    NSPasteboard* pb = [NSPasteboard generalPasteboard];
    NSArray<NSURL*>* urls = [pb readObjectsForClasses:@[ [NSURL class] ] options:@{NSPasteboardURLReadingFileURLsOnlyKey : @YES}];
    for (NSURL* url in urls) {
        std::string ext = std::filesystem::path(url.fileSystemRepresentation).extension().string();
        if (!ext.empty()) ext.erase(0, 1);
        for (auto& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (std::ranges::find(photo_extensions(), ext) != photo_extensions().end()) out.files.emplace_back(url.fileSystemRepresentation);
    }
    if (!out.files.empty()) return out;
    for (NSString* type in @[ @"public.jpeg", @"public.heic", @"public.png", @"public.tiff" ]) {
        NSData* data = [pb dataForType:type];
        if (data.length > 0) {
            out.image_bytes.assign(static_cast<const char*>(data.bytes), data.length);
            break;
        }
    }
    return out;
}

std::optional<std::filesystem::path> choose_save_file(const std::string& name, const std::string& extension) {
    NSSavePanel* panel = [NSSavePanel savePanel];
    panel.nameFieldStringValue = [NSString stringWithUTF8String:name.c_str()];
    panel.allowedContentTypes = types_of({extension});
    [NSApp activateIgnoringOtherApps:YES];  // background-launched: bring the panel to the front, keyed
    if ([panel runModal] != NSModalResponseOK || !panel.URL) return std::nullopt;
    return std::filesystem::path(panel.URL.fileSystemRepresentation);
}

}  // namespace einstar::modelapp
