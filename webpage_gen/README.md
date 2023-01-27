# Static Webpage Generator

Author: Michael Crum

My custom Node.js based static site generator for github pages.

Static site generation is a simple process, yet prebuilt tools tend to be bloated with dependencies, a pain to learn, or lacking in fine grained customization. So I decided to make my own! It's simple, but gets the job done.

Any folder in the `/posts` directory will create a new page from `content.md` and populate the sidebar with info from the relevant `info.json` file. Templates for posts, the home page, and sidebar elements are found in the `/templates` directory, and styles are located in `/styles`. See `/posts/lab_1` for an example post folder.

Run `node.js index.js` to generate the site, which will be placed in ../docs. This is traditionally where github pages serves from, just enable pages in the repository settings. Simply build and push to github, and your website will update accordingly.