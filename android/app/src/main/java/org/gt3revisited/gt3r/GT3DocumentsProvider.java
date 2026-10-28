package org.gt3revisited.gt3r;

import android.annotation.TargetApi;
import android.content.Context;
import android.content.res.AssetFileDescriptor;
import android.database.Cursor;
import android.database.MatrixCursor;
import android.graphics.Point;
import android.os.Build;
import android.os.CancellationSignal;
import android.os.ParcelFileDescriptor;
import android.provider.DocumentsContract;
import android.provider.DocumentsContract.Document;
import android.provider.DocumentsContract.Root;
import android.provider.DocumentsProvider;
import android.webkit.MimeTypeMap;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.LinkedList;
import java.util.Locale;

/**
 * Shows the game's user data folder (the app's files dir, SDL_GetPrefPath: config.ini, mods/,
 * screenshots) in the system Files app and file pickers as "GT3 Revisited", so players can
 * copy mod folders in and screenshots out. Android keeps app storage private otherwise, even
 * from file managers (issue 22). Document ids are "gt3r" for the folder itself and
 * "gt3r/<path inside it>" below it.
 */
public class GT3DocumentsProvider extends DocumentsProvider {
    static final String ROOT_ID = "gt3r";

    private static final String[] ROOT_COLUMNS = {
        Root.COLUMN_ROOT_ID, Root.COLUMN_FLAGS, Root.COLUMN_ICON, Root.COLUMN_TITLE, Root.COLUMN_SUMMARY,
        Root.COLUMN_DOCUMENT_ID, Root.COLUMN_AVAILABLE_BYTES, Root.COLUMN_MIME_TYPES,
    };
    private static final String[] DOCUMENT_COLUMNS = {
        Document.COLUMN_DOCUMENT_ID, Document.COLUMN_MIME_TYPE, Document.COLUMN_DISPLAY_NAME,
        Document.COLUMN_LAST_MODIFIED, Document.COLUMN_FLAGS, Document.COLUMN_SIZE,
    };

    private File base;

    static String authority(Context c) {
        return c.getPackageName() + ".documents";
    }

    @Override
    public boolean onCreate() {
        base = getContext().getFilesDir();
        return true;
    }

    @Override
    public Cursor queryRoots(String[] projection) {
        MatrixCursor c = new MatrixCursor(projection != null ? projection : ROOT_COLUMNS);
        MatrixCursor.RowBuilder row = c.newRow();
        row.add(Root.COLUMN_ROOT_ID, ROOT_ID);
        row.add(Root.COLUMN_DOCUMENT_ID, ROOT_ID);
        row.add(Root.COLUMN_TITLE, getContext().getString(R.string.app_name));
        row.add(Root.COLUMN_SUMMARY, getContext().getString(R.string.documents_summary));
        row.add(Root.COLUMN_FLAGS, Root.FLAG_SUPPORTS_CREATE | Root.FLAG_SUPPORTS_IS_CHILD | Root.FLAG_LOCAL_ONLY);
        row.add(Root.COLUMN_ICON, R.mipmap.ic_launcher);
        row.add(Root.COLUMN_MIME_TYPES, "*/*");
        row.add(Root.COLUMN_AVAILABLE_BYTES, base.getUsableSpace());
        return c;
    }

    @Override
    public Cursor queryDocument(String documentId, String[] projection) throws FileNotFoundException {
        MatrixCursor c = new MatrixCursor(projection != null ? projection : DOCUMENT_COLUMNS);
        addRow(c, fileFor(documentId));
        return c;
    }

    @Override
    public Cursor queryChildDocuments(String parentId, String[] projection, String sortOrder) throws FileNotFoundException {
        File dir = fileFor(parentId);
        MatrixCursor c = new MatrixCursor(projection != null ? projection : DOCUMENT_COLUMNS);
        File[] children = dir.listFiles();
        if (children != null) {
            for (File f : children) {
                if (!f.getName().startsWith(".")) addRow(c, f); // the game's own work folders
            }
        }
        c.setNotificationUri(getContext().getContentResolver(), childrenUri(parentId));
        return c;
    }

    @Override
    public ParcelFileDescriptor openDocument(String documentId, String mode, CancellationSignal signal)
            throws FileNotFoundException {
        File f = fileFor(documentId);
        if (f.isDirectory()) throw new FileNotFoundException(documentId + " is a folder");
        return ParcelFileDescriptor.open(f, ParcelFileDescriptor.parseMode(mode));
    }

    @Override
    public AssetFileDescriptor openDocumentThumbnail(String documentId, Point sizeHint, CancellationSignal signal)
            throws FileNotFoundException {
        // Screenshots: the Files app scales the picture itself.
        ParcelFileDescriptor pfd = ParcelFileDescriptor.open(fileFor(documentId), ParcelFileDescriptor.MODE_READ_ONLY);
        return new AssetFileDescriptor(pfd, 0, AssetFileDescriptor.UNKNOWN_LENGTH);
    }

    @Override
    public String createDocument(String parentId, String mimeType, String displayName) throws FileNotFoundException {
        File dir = fileFor(parentId);
        File f = unusedFile(dir, cleanName(displayName));
        boolean ok;
        try {
            ok = Document.MIME_TYPE_DIR.equals(mimeType) ? f.mkdir() : f.createNewFile();
        } catch (IOException e) {
            ok = false;
        }
        if (!ok) throw new FileNotFoundException("Cannot create " + f.getName());
        notifyChildren(parentId);
        return idFor(f);
    }

    @Override
    public void deleteDocument(String documentId) throws FileNotFoundException {
        File f = fileFor(documentId);
        if (isBase(f)) throw new FileNotFoundException("The GT3 Revisited folder itself stays");
        if (!deleteTree(f)) throw new FileNotFoundException("Cannot delete " + f.getName());
        notifyChildren(idFor(f.getParentFile()));
    }

    @Override
    public void removeDocument(String documentId, String parentDocumentId) throws FileNotFoundException {
        deleteDocument(documentId); // every document has one parent
    }

    @Override
    public String renameDocument(String documentId, String displayName) throws FileNotFoundException {
        File f = fileFor(documentId);
        if (isBase(f)) throw new FileNotFoundException("The GT3 Revisited folder itself stays");
        File to = new File(f.getParentFile(), cleanName(displayName));
        if (to.exists() || !f.renameTo(to)) throw new FileNotFoundException("Cannot rename " + f.getName());
        notifyChildren(idFor(f.getParentFile()));
        return idFor(to);
    }

    @Override
    public String moveDocument(String sourceDocumentId, String sourceParentDocumentId, String targetParentDocumentId)
            throws FileNotFoundException {
        File f = fileFor(sourceDocumentId), dir = fileFor(targetParentDocumentId);
        if (isBase(f)) throw new FileNotFoundException("The GT3 Revisited folder itself stays");
        File to = new File(dir, f.getName());
        if (to.exists() || contains(f, dir) || !f.renameTo(to)) throw new FileNotFoundException("Cannot move " + f.getName());
        notifyChildren(idFor(f.getParentFile()));
        notifyChildren(targetParentDocumentId);
        return idFor(to);
    }

    /** The ids from `parentDocumentId` (or the root) down to the document: the Files app opens a folder by it. */
    @Override
    @TargetApi(Build.VERSION_CODES.O)
    public DocumentsContract.Path findDocumentPath(String parentDocumentId, String childDocumentId)
            throws FileNotFoundException {
        File child = fileFor(childDocumentId), top = parentDocumentId != null ? fileFor(parentDocumentId) : base;
        if (!contains(top, child)) throw new FileNotFoundException(childDocumentId + " is not in " + parentDocumentId);
        LinkedList<String> ids = new LinkedList<>();
        for (File f = child; ; f = f.getParentFile()) {
            ids.addFirst(idFor(f));
            if (f.getAbsolutePath().equals(top.getAbsolutePath())) break;
        }
        return new DocumentsContract.Path(parentDocumentId != null ? null : ROOT_ID, ids);
    }

    @Override
    public boolean isChildDocument(String parentDocumentId, String documentId) {
        try {
            return contains(fileFor(parentDocumentId), fileFor(documentId));
        } catch (FileNotFoundException e) {
            return false;
        }
    }

    // --- paths ---

    /** The file of a document id; refuses ids that lead outside the user data folder. */
    private File fileFor(String documentId) throws FileNotFoundException {
        File f;
        if (documentId.equals(ROOT_ID)) f = base;
        else if (documentId.startsWith(ROOT_ID + "/")) f = new File(base, documentId.substring(ROOT_ID.length() + 1));
        else throw new FileNotFoundException(documentId);
        if (!contains(base, f)) throw new FileNotFoundException(documentId);
        if (!f.exists()) throw new FileNotFoundException(documentId);
        return f;
    }

    private String idFor(File f) {
        String root = base.getAbsolutePath(), path = f.getAbsolutePath();
        return path.equals(root) ? ROOT_ID : ROOT_ID + "/" + path.substring(root.length() + 1);
    }

    private boolean isBase(File f) {
        return f.getAbsolutePath().equals(base.getAbsolutePath());
    }

    /** Whether `f` is `dir` or inside it, after resolving "..". */
    private static boolean contains(File dir, File f) {
        try {
            String d = dir.getCanonicalPath(), p = f.getCanonicalPath();
            return p.equals(d) || p.startsWith(d + File.separator);
        } catch (IOException e) {
            return false;
        }
    }

    private static String cleanName(String name) throws FileNotFoundException {
        String n = name == null ? "" : name.replace('/', '_').replace('\0', '_').trim();
        if (n.isEmpty() || n.equals(".") || n.equals("..")) throw new FileNotFoundException("Not a usable name: " + name);
        return n;
    }

    /** `name` in `dir`, or "name (2).ext" and so on when it is taken. */
    private static File unusedFile(File dir, String name) {
        File f = new File(dir, name);
        int dot = name.lastIndexOf('.');
        String stem = dot > 0 ? name.substring(0, dot) : name, ext = dot > 0 ? name.substring(dot) : "";
        for (int i = 2; f.exists(); ++i) f = new File(dir, stem + " (" + i + ")" + ext);
        return f;
    }

    /** Deletes `f` and, for a folder, everything in it (a symbolic link itself, not its target). */
    private static boolean deleteTree(File f) {
        boolean link;
        try {
            link = !f.getCanonicalFile().equals(new File(f.getParentFile().getCanonicalFile(), f.getName()));
        } catch (IOException e) {
            link = true;
        }
        File[] children = f.isDirectory() && !link ? f.listFiles() : null;
        if (children != null) {
            for (File c : children) deleteTree(c);
        }
        return f.delete();
    }

    private static String mimeOf(File f) {
        if (f.isDirectory()) return Document.MIME_TYPE_DIR;
        String name = f.getName();
        int dot = name.lastIndexOf('.');
        String ext = dot >= 0 ? name.substring(dot + 1).toLowerCase(Locale.ROOT) : "";
        switch (ext) {
            case "ini": case "txt": case "md": case "log": case "cfg": return "text/plain";
            default: break;
        }
        String mime = ext.isEmpty() ? null : MimeTypeMap.getSingleton().getMimeTypeFromExtension(ext);
        return mime != null ? mime : "application/octet-stream";
    }

    private void addRow(MatrixCursor c, File f) {
        String mime = mimeOf(f);
        int flags = f.isDirectory() ? Document.FLAG_DIR_SUPPORTS_CREATE : Document.FLAG_SUPPORTS_WRITE;
        if (!isBase(f)) {
            flags |= Document.FLAG_SUPPORTS_DELETE | Document.FLAG_SUPPORTS_RENAME | Document.FLAG_SUPPORTS_MOVE
                    | Document.FLAG_SUPPORTS_REMOVE;
        }
        if (mime.startsWith("image/")) flags |= Document.FLAG_SUPPORTS_THUMBNAIL;
        MatrixCursor.RowBuilder row = c.newRow();
        row.add(Document.COLUMN_DOCUMENT_ID, idFor(f));
        row.add(Document.COLUMN_DISPLAY_NAME, isBase(f) ? getContext().getString(R.string.app_name) : f.getName());
        row.add(Document.COLUMN_SIZE, f.isDirectory() ? null : f.length());
        row.add(Document.COLUMN_MIME_TYPE, mime);
        row.add(Document.COLUMN_LAST_MODIFIED, f.lastModified());
        row.add(Document.COLUMN_FLAGS, flags);
    }

    private android.net.Uri childrenUri(String parentId) {
        return DocumentsContract.buildChildDocumentsUri(authority(getContext()), parentId);
    }

    private void notifyChildren(String parentId) {
        getContext().getContentResolver().notifyChange(childrenUri(parentId), null);
    }
}
