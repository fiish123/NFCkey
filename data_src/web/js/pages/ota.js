(function() {
    const PACKAGE_EXTENSION = '.zip';
    const FIRMWARE_ENTRY_NAME = 'firmware.bin';
    const FILESYSTEM_PREFIX = 'data/';
    const MAX_FIRMWARE_SIZE = 2 * 1024 * 1024;

    let selectedFile = null;
    let otaSystemInfo = null;
    let systemInfoPromise = null;
    let updateRunning = false;

    function init() {
        loadSystemInfo();
        initFileUpload();

        // WebSocket 重连后自动刷新系统信息（去抖：2s 内的重复事件忽略）
        let lastWsRefreshTs = 0;
        document.addEventListener('ws-connected', function() {
            const now = Date.now();
            if (now - lastWsRefreshTs < 2000) return;
            lastWsRefreshTs = now;
            loadSystemInfo();
        });
    }

    function loadSystemInfo() {
        if (systemInfoPromise) {
            return systemInfoPromise;
        }

        systemInfoPromise = fetch('/api/system/info')
            .then(response => {
                if (!response.ok) throw new Error('HTTP ' + response.status);
                return response.json();
            })
            .then(response => {
                if (!response.success || !response.data) {
                    throw new Error(response.message || '未知错误');
                }

                const data = response.data;
                otaSystemInfo = data;
                document.getElementById('chipModel').textContent = data.chipModel || '未知';
                document.getElementById('chipId').textContent = data.chipId || '未知';
                document.getElementById('currentVersion').textContent = data.version || '未知';
                return data;
            })
            .catch(err => {
                console.error('获取系统信息失败:', err);
                otaSystemInfo = null;
                document.getElementById('chipModel').textContent = '获取失败';
                document.getElementById('chipId').textContent = '-';
                document.getElementById('currentVersion').textContent = '-';
                throw err;
            })
            .finally(() => {
                systemInfoPromise = null;
            });

        return systemInfoPromise;
    }

    function initFileUpload() {
        const uploadBox = document.getElementById('uploadBox');
        const fileInput = document.getElementById('file');
        const uploadBoxContent = document.getElementById('uploadBoxContent');
        const fileInfo = document.getElementById('fileInfo');
        const fileName = document.getElementById('fileName');
        const fileSize = document.getElementById('fileSize');

        if (!uploadBox || !fileInput) return;

        bindFilePickerTrigger(uploadBox, fileInput);
        initDragAndDrop(uploadBox, fileInput, function(file) {
            if (file) {
                selectedFile = file;
                displayFileInfo(uploadBoxContent, fileInfo, fileName, fileSize, file);
                validateAndUpload(file);
            }
        });
    }

    function resetSelectedUploadArea() {
        const uploadBoxContent = document.getElementById('uploadBoxContent');
        const fileInfo = document.getElementById('fileInfo');
        const fileInput = document.getElementById('file');

        resetUploadArea(uploadBoxContent, fileInfo, fileInput);
        selectedFile = null;
        updateRunning = false;
    }

    function setBusyState(busy) {
        updateRunning = busy;
        const uploadBox = document.getElementById('uploadBox');
        const fileInput = document.getElementById('file');

        if (uploadBox) {
            uploadBox.setAttribute('aria-disabled', busy ? 'true' : 'false');
            uploadBox.style.pointerEvents = busy ? 'none' : '';
            uploadBox.style.opacity = busy ? '0.7' : '';
        }

        if (fileInput) {
            fileInput.disabled = busy;
        }
    }

    async function validateAndUpload(file) {
        if (updateRunning) {
            showToast('升级正在进行中，请稍候', 'warning');
            return;
        }

        if (!file.name.toLowerCase().endsWith(PACKAGE_EXTENSION)) {
            showToast('请选择 .zip 格式的升级包', 'error');
            resetSelectedUploadArea();
            return;
        }

        if (!window.fflate || typeof window.fflate.unzipSync !== 'function') {
            showToast('浏览器端压缩包解析模块未就绪', 'error');
            resetSelectedUploadArea();
            return;
        }

        // 升级前二次确认（防误操作导致设备变砖）。确认必须在任何进度副作用开始前解析。
        const confirmed = await showConfirm('升级过程中设备将重启，期间请勿断电或关闭页面，否则可能导致设备变砖。确认开始升级？', {
            type: 'danger',
            title: '确认升级',
            confirmText: '开始升级',
            cancelText: '取消'
        });
        if (!confirmed) {
            resetSelectedUploadArea();
            return;
        }

        startPackageUpdate(file);
    }

    async function startPackageUpdate(file) {
        setBusyState(true);
        showProgress(true);
        updateStage('正在解析升级包...');
        updateProgress(2);

        try {
            if (!otaSystemInfo) {
                await loadSystemInfo();
            }

            const packageInfo = await extractPackage(file);
            renderPackageSummary(packageInfo);

            updateStage('正在校验文件系统文件...');
            updateProgress(10);
            const syncSummary = await syncFilesystem(packageInfo.filesystemFiles);
            renderSyncSummary(syncSummary);

            updateStage('正在上传固件，设备稍后会重启...');
            updateProgress(80);
            await uploadFirmware(packageInfo.firmware);

            updateStage('升级完成，设备正在重启...');
            updateProgress(100);
            showToast('升级完成，设备即将重启', 'success');
            const uploadBox = document.getElementById('uploadBox');
            if (uploadBox) {
                uploadBox.style.display = 'none';
            }
        } catch (err) {
            console.error('升级失败:', err);
            updateStage('升级失败');
            showProgress(false);
            showToast(err.message || '升级失败', 'error');
            setBusyState(false);
            return;
        }

        setBusyState(false);
    }

    async function extractPackage(file) {
        const zipBytes = new Uint8Array(await file.arrayBuffer());
        let entries;

        try {
            entries = window.fflate.unzipSync(zipBytes);
        } catch (err) {
            throw new Error('升级包解析失败，请确认 ZIP 文件有效');
        }

        let firmware = null;
        const filesystemFiles = [];

        Object.entries(entries).forEach(([entryName, entryData]) => {
            const normalizedEntry = entryName.replace(/\\/g, '/');
            if (!entryData || normalizedEntry.endsWith('/')) {
                return;
            }

            if (normalizedEntry === FIRMWARE_ENTRY_NAME || normalizedEntry.endsWith('/' + FIRMWARE_ENTRY_NAME)) {
                firmware = {
                    name: normalizedEntry.split('/').pop(),
                    data: entryData,
                    size: entryData.length
                };
                return;
            }

            const relativePath = extractFilesystemRelativePath(normalizedEntry);
            if (!relativePath) {
                return;
            }

            filesystemFiles.push({
                archivePath: normalizedEntry,
                targetPath: '/' + relativePath,
                filename: relativePath.split('/').pop(),
                data: entryData,
                size: entryData.length
            });
        });

        if (!firmware) {
            throw new Error('升级包中缺少 firmware.bin');
        }

        if (firmware.size > MAX_FIRMWARE_SIZE) {
            throw new Error('固件文件过大，超过当前页面允许的 2MB 限制');
        }

        return {
            firmware: firmware,
            filesystemFiles: filesystemFiles,
            archiveSize: file.size
        };
    }

    function extractFilesystemRelativePath(entryPath) {
        if (entryPath.startsWith(FILESYSTEM_PREFIX)) {
            return entryPath.slice(FILESYSTEM_PREFIX.length).replace(/^\/+/, '');
        }

        const nestedPrefix = '/' + FILESYSTEM_PREFIX;
        const nestedIndex = entryPath.indexOf(nestedPrefix);
        if (nestedIndex >= 0) {
            return entryPath.slice(nestedIndex + nestedPrefix.length).replace(/^\/+/, '');
        }

        return '';
    }

    async function syncFilesystem(filesystemFiles) {
        if (!filesystemFiles.length) {
            updateProgress(75);
            return { uploaded: 0, skipped: 0, preserved: 0 };
        }

        const checkPayload = [];
        for (let i = 0; i < filesystemFiles.length; i++) {
            const file = filesystemFiles[i];
            updateStage(`正在计算文件哈希 (${i + 1}/${filesystemFiles.length})...`);
            updateProgress(10 + Math.round(((i + 1) / filesystemFiles.length) * 20));
            file.sha256 = await computeBufferSha256(file.data);
            checkPayload.push({
                path: file.targetPath,
                size: file.size,
                sha256: file.sha256
            });
        }

        const checkResponse = await fetch('/api/files/sync-check', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ files: checkPayload })
        });

        const checkJson = await parseApiResponse(checkResponse, '文件系统校验失败');
        const results = Array.isArray(checkJson.data && checkJson.data.results) ? checkJson.data.results : [];
        const resultMap = new Map(results.map(item => [item.path, item]));

        let uploaded = 0;
        let skipped = 0;
        let preserved = 0;
        const uploadTargets = [];

        for (const file of filesystemFiles) {
            const result = resultMap.get(file.targetPath);
            if (!result) {
                throw new Error('设备未返回文件校验结果: ' + file.targetPath);
            }

            if (result.action === 'invalid') {
                throw new Error('升级包中的文件路径无效: ' + file.targetPath);
            }

            if (result.action === 'preserve') {
                preserved += 1;
                continue;
            }

            if (result.action === 'skip') {
                skipped += 1;
                continue;
            }

            uploadTargets.push(file);
        }

        for (let i = 0; i < uploadTargets.length; i++) {
            const file = uploadTargets[i];
            updateStage(`正在同步文件系统 (${i + 1}/${uploadTargets.length})...`);
            updateProgress(30 + Math.round(((i + 1) / Math.max(uploadTargets.length, 1)) * 45));
            await uploadFilesystemFile(file);
            uploaded += 1;
        }

        return { uploaded, skipped, preserved };
    }

    async function uploadFilesystemFile(file) {
        const formData = new FormData();
        formData.append('file', new Blob([file.data], { type: 'application/octet-stream' }), file.filename);

        const parentDirectory = getParentDirectory(file.targetPath);
        const headerName = otaSystemInfo && otaSystemInfo.fileHashHeader;

        await sendMultipartRequest('/api/files?path=' + encodeURIComponent(parentDirectory), formData, headerName ? {
            [headerName]: file.sha256
        } : {});
    }

    async function uploadFirmware(firmware) {
        const formData = new FormData();
        formData.append('file', new Blob([firmware.data], { type: 'application/octet-stream' }), firmware.name);

        const hashHeaders = {};
        if (otaSystemInfo && otaSystemInfo.otaHashAlgorithm === 'sha256' && otaSystemInfo.otaHashHeader) {
            hashHeaders[otaSystemInfo.otaHashHeader] = await computeBufferSha256(firmware.data);
        }

        await sendMultipartRequest('/api/system/firmware', formData, hashHeaders, function(percent) {
            updateProgress(80 + Math.round(percent * 0.2));
        });
    }

    async function sendMultipartRequest(url, formData, headers, onProgress) {
        return new Promise((resolve, reject) => {
            const xhr = new XMLHttpRequest();
            xhr.open('POST', url, true);

            Object.entries(headers || {}).forEach(([headerName, headerValue]) => {
                xhr.setRequestHeader(headerName, headerValue);
            });

            xhr.upload.onprogress = function(e) {
                if (e.lengthComputable && typeof onProgress === 'function') {
                    const percent = Math.round((e.loaded / e.total) * 100);
                    onProgress(percent);
                }
            };

            xhr.onload = function() {
                if (xhr.status >= 200 && xhr.status < 300) {
                    resolve(xhr.responseText);
                    return;
                }

                reject(new Error(extractErrorMessage(xhr.status, xhr.responseText)));
            };

            xhr.onerror = function() {
                reject(new Error('网络错误'));
            };

            xhr.send(formData);
        });
    }

    async function parseApiResponse(response, fallbackMessage) {
        let payload = null;
        try {
            payload = await response.json();
        } catch (err) {
            throw new Error(fallbackMessage);
        }

        if (!response.ok || !payload.success) {
            throw new Error(payload.message || fallbackMessage || ('HTTP ' + response.status));
        }

        return payload;
    }

    function extractErrorMessage(status, responseText) {
        let errorMsg = 'HTTP ' + status;

        if (!responseText) {
            return errorMsg;
        }

        try {
            const jsonData = JSON.parse(responseText);
            return jsonData.message || jsonData.data?.error || jsonData.data?.reason || errorMsg;
        } catch (err) {
            return responseText;
        }
    }

    function getParentDirectory(path) {
        const normalized = path.replace(/\/+/g, '/');
        const separatorIndex = normalized.lastIndexOf('/');
        if (separatorIndex <= 0) {
            return '/';
        }

        return normalized.slice(0, separatorIndex) || '/';
    }

    function renderPackageSummary(packageInfo) {
        const packageSummary = document.getElementById('packageSummary');
        if (packageSummary) {
            packageSummary.style.display = 'block';
        }

        document.getElementById('packageFirmwareName').textContent = packageInfo.firmware.name;
        document.getElementById('packageFirmwareSize').textContent = formatFileSize(packageInfo.firmware.size);
        document.getElementById('packageFsCount').textContent = String(packageInfo.filesystemFiles.length);
        document.getElementById('packageSyncResult').textContent = '等待同步';
    }

    function renderSyncSummary(syncSummary) {
        const summaryText = `上传 ${syncSummary.uploaded} 个，跳过 ${syncSummary.skipped} 个，保留 ${syncSummary.preserved} 个`;
        document.getElementById('packageSyncResult').textContent = summaryText;
    }

    function showProgress(visible) {
        const progressContainer = document.getElementById('progressContainer');
        if (progressContainer) {
            progressContainer.style.display = visible ? 'block' : 'none';
        }
    }

    function updateStage(text) {
        const progressStage = document.getElementById('progressStage');
        if (progressStage) {
            progressStage.textContent = text;
        }
    }

    function updateProgress(percent) {
        const progressBar = document.getElementById('progressBar');
        if (!progressBar) {
            return;
        }

        const safePercent = Math.max(0, Math.min(100, percent));
        progressBar.style.width = safePercent + '%';
        progressBar.textContent = safePercent + '%';
    }

    window.addEventListener('DOMContentLoaded', init);
})();
