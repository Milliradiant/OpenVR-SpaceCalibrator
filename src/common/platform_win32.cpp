#include "platform.h"

#if OS_WINDOWS
#include "constants.h"
#include "log.h"
#include <filesystem>

#define WIN32_LEAN_AND_MEAN
#include <combaseapi.h>
#include <knownfolders.h>
#include <shellapi.h>
#include <shlobj.h>
#include <windows.h>

namespace platform {
std::filesystem::path getUserConfigDir()
{
    PWSTR path_pwstr = NULL;
    HRESULT hr = SHGetKnownFolderPath(FOLDERID_RoamingAppData, 0, NULL, &path_pwstr);

    if (SUCCEEDED(hr)) {
        auto thePath = std::filesystem::path(path_pwstr);
        CoTaskMemFree(path_pwstr);
        return thePath;
    }

    return std::filesystem::path();
}

std::filesystem::path getExeDir()
{
    wchar_t path_buf[MAX_PATH] = {};
    DWORD size = GetModuleFileNameW(NULL, path_buf, MAX_PATH);
    if (size > 0 && size <= MAX_PATH) {
        return std::filesystem::path(path_buf).parent_path();
    }

    return std::filesystem::path();
}

// %LOCALAPPDATA%/openvr/openvrpaths.vrpath
std::filesystem::path getSteamvrVrPathsPath()
{

    PWSTR path_pwstr = NULL;
    HRESULT hr = SHGetKnownFolderPath(FOLDERID_LocalAppData, 0, NULL, &path_pwstr);
    if (SUCCEEDED(hr)) {
        auto thePath = std::filesystem::path(path_pwstr) / "openvr" / "openvrpaths.vrpath";
        CoTaskMemFree(path_pwstr);
        return thePath;
    }

    return std::filesystem::path();
}

std::string getEnvVariable(const std::string& szEnvVarName)
{
    char szStringBuffer[256] = {};
    DWORD result = GetEnvironmentVariableA(szEnvVarName.c_str(), szStringBuffer, sizeof(szStringBuffer));
    if (result > 0 && szStringBuffer[0] != 0) {
        return szStringBuffer;
    }
    return "";
}

constexpr const char* s_STEAM_MUTEX_KEY = "Global\\MUTEX__SpaceCalibrator_Steam";
HANDLE s_hSteamMutex = INVALID_HANDLE_VALUE;
bool s_isGitHubVersionInstalled = false;

bool isAnotherInstanceRunning(bool& bIsRunningViaSteam)
{

    bIsRunningViaSteam = false;
    std::string szSteamAppIdEnv = getEnvVariable("SteamAppId");
    if (spacecal::c_SPACE_CALIBRATOR_STEAM_APP_ID == szSteamAppIdEnv || spacecal::c_STEAMVR_STEAM_APP_ID == szSteamAppIdEnv) {
        // We got launched via the Steam client UI.
        s_hSteamMutex = CreateMutexA(NULL, FALSE, s_STEAM_MUTEX_KEY);
        bIsRunningViaSteam = true;
        if (s_hSteamMutex == nullptr) {
            s_hSteamMutex = INVALID_HANDLE_VALUE;
        } else {
            // mutex opened, check if we opened one, if so exit
            if (GetLastError() == ERROR_ALREADY_EXISTS) {
                CloseHandle(s_hSteamMutex);
                s_hSteamMutex = INVALID_HANDLE_VALUE;
                return true;
            }
        }
        return false;
    }

    return false;
}
void shutdownCurrentInstance()
{
    if (s_hSteamMutex != INVALID_HANDLE_VALUE && s_hSteamMutex != nullptr) {
        CloseHandle(s_hSteamMutex);
        s_hSteamMutex = nullptr;
    }
}

void showMessageDialog(const std::string& title, const std::string& message)
{
    LOG_INFO("displaying dialog [{}]: {}", title, message);
    MessageBoxA(nullptr, message.c_str(), title.c_str(), 0);
}

void launchDirInFileBrowser(const std::filesystem::path& szDirectory)
{
    PIDLIST_ABSOLUTE nativeFolder = ILCreateFromPathW(szDirectory.c_str());
    if (!nativeFolder) {
        return;
    }

    PCUITEMID_CHILD_ARRAY fileArray = (PCUITEMID_CHILD_ARRAY)&nativeFolder;
    HRESULT hr = SHOpenFolderAndSelectItems(nativeFolder, 1, fileArray, 0);

    if (FAILED(hr)) {
        ShellExecuteW(NULL, L"open", L"explorer.exe", szDirectory.c_str(), NULL, SW_SHOWNORMAL);
    }

    CoTaskMemFree(nativeFolder);
}

void launchWebpage(const std::string& szUrl)
{
    // https://www.betaarchive.com/wiki/index.php/Microsoft_KB_Archive/224816#How_ShellExecute_Interprets_the_URL_Passed
    ShellExecuteA(NULL, "open", szUrl.c_str(), NULL, NULL, SW_SHOWNORMAL);
}

void setThreadName(const std::string& threadName)
{
    std::wstring szWinThreadName(threadName.size(), L' ');
    szWinThreadName.resize(std::mbstowcs(&szWinThreadName[0], threadName.c_str(), threadName.size()));
    SetThreadDescription(GetCurrentThread(), szWinThreadName.c_str());
}

bool mapFileReadOnly(const std::filesystem::path& path, MappedFile_t& file)
{
    file = {};
    HANDLE hFile = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hFile == INVALID_HANDLE_VALUE) {
        return false;
    }
    LARGE_INTEGER size = {};
    if (!GetFileSizeEx(hFile, &size) || size.QuadPart <= 0) {
        CloseHandle(hFile);
        return false;
    }
    HANDLE hMapping = CreateFileMappingW(hFile, NULL, PAGE_READONLY, 0, 0, NULL);
    CloseHandle(hFile); // the mapping keeps the file open
    if (hMapping == NULL) {
        return false;
    }
    const void* pData = MapViewOfFile(hMapping, FILE_MAP_READ, 0, 0, 0);
    if (pData == NULL) {
        CloseHandle(hMapping);
        return false;
    }
    file.pData = pData;
    file.dwSize = (size_t)size.QuadPart;
    file.hMapping = hMapping;
    return true;
}

void unmapFile(MappedFile_t& file)
{
    if (file.pData) {
        UnmapViewOfFile(file.pData);
    }
    if (file.hMapping) {
        CloseHandle((HANDLE)file.hMapping);
    }
    file = {};
}
}
#endif
