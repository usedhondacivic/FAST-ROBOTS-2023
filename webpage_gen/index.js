const fs = require('fs');
const path = require('path');
const fsExtra = require('fs-extra');
const glob = require("glob");
const sharp = require("sharp");
var showdown = require('showdown'),
    converter = new showdown.Converter();

// Grab the templates
const index_template = fs.readFileSync(path.resolve(__dirname, 'templates/index.html'), 'utf8');
const homepage_entry_template = fs.readFileSync(path.resolve(__dirname, 'templates/homepage_entry.html'), 'utf8');
const sidebar_entry_template = fs.readFileSync(path.resolve(__dirname, 'templates/sidebar_entry.html'), 'utf8');
const content_template = fs.readFileSync(path.resolve(__dirname, 'templates/content.html'), 'utf8');

// Clean the docs directory
fsExtra.emptyDirSync(path.resolve(__dirname, '../docs'));

// Copy over styles
fsExtra.copySync(path.resolve(__dirname, 'styles'), path.resolve(__dirname, '../docs/styles'))

var pages = new Object();
var info_arr = [];
var sidebar = [];
var homepage_entries = [];

// Copy a folder of images, and downsize them to reduce loading times.
function resize_and_relocate(src_dir, target_dir) {
    fs.mkdirSync(target_dir, { recursive: true });
    glob(src_dir + "/**", function (er, files) {
        files.forEach(path_name => {
            let file_split = path_name.split("/");
            let file_name = file_split[file_split.length - 1];
            if (file_name == "assets") return;
            sharp(path_name)
                .resize({
                    width: 900,
                    height: 600,
                    fit: sharp.fit.inside
                })
                .toFile(target_dir + "/" + file_name)
        })
    })
}

// Iterate over all of the posts
glob("posts/*/*.md", function (er, files) {
    files.forEach(path_name => {
        var info = JSON.parse(fs.readFileSync(path.resolve(__dirname, path_name.replace("content.md", "info.json"))));
        // Populate the content of a project page
        var contents_md = fs.readFileSync(path.resolve(__dirname, path_name), 'utf8');
        var contents_html = converter.makeHtml(contents_md);
        var contents_page = content_template.replaceAll("<!-- TITLE -->", info.title)
            .replace("<!-- CONTENT -->", contents_html)
            .replace("<!-- DATE -->", info.date);
        var new_path = path_name.replace("posts", "../docs").replace("/content.md", "");
        pages[new_path] = contents_page;
        // Copy assets
        resize_and_relocate(path_name.replace("/content.md", "/assets"), new_path + '/assets');
        info.rel_post_link = new_path.replace("../docs", "");
        info.rel_snapshot_link = new_path.replace("../docs", "") + "/assets/snapshot.png";
        info_arr.push(info);
    })
    info_arr.sort(function (a, b) {
        return new Date(a.date) - new Date(b.date);
    })
    info_arr.forEach(info => {
        // Create sidebar entry
        var sidebar_entry = sidebar_entry_template.replace("<!-- TITLE -->", info.title)
            .replace("<!-- DATE -->", info.date)
            .replaceAll("project_link", ".." + info.rel_post_link)
            .replace("snapshot_link", ".." + info.rel_snapshot_link);
        sidebar.push(sidebar_entry);
        // Create home page entry
        var homepage_entry = homepage_entry_template.replace("<!-- TITLE -->", info.title)
            .replace("<!-- DATE -->", info.date)
            .replace("<!-- DESCRIPTION -->", info.description)
            .replace("<!-- TECH -->", info.tech ? "Technologies used: " + info.tech : "")
            .replace("<!-- TOOLS -->", info.tools ? "Tools used: " + info.tools : "")
            .replaceAll("post_link", "." + info.rel_post_link)
            .replace("snapshot_link", "." + info.rel_snapshot_link);
        homepage_entries.push(homepage_entry);
    })
    // Write the html files for each project to the docs folder
    for (const [path_loc, html] of Object.entries(pages)) {
        var content = html.replace("<!-- NAV -->", sidebar.join("\n"));
        fs.mkdirSync(path_loc, { recursive: true });
        fs.writeFileSync(path.resolve(__dirname, path_loc, "index.html"), content);
    }
    // Write the home page to the docs folder
    var content = index_template.replace("<!-- PROJECTS -->", homepage_entries.join("\n"));
    fs.writeFileSync(path.resolve(__dirname, "../docs/index.html"), content);
    // Write global assets to the docs folder
    fsExtra.copySync(path.resolve(__dirname, "global_assets"), path.resolve(__dirname, "../docs/global_assets"));
})