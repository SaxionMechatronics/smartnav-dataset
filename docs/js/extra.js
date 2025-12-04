document$.subscribe(function () {
    // Initialize syntax highlighting
    if (typeof hljs !== 'undefined') {
        document.querySelectorAll('pre code').forEach((block) => {
            hljs.highlightBlock(block);
        });
    }

    // Fix for nested lists with code blocks
    const codeBlocksInLists = document.querySelectorAll('li pre');
    codeBlocksInLists.forEach((block) => {
        block.style.marginTop = '1em';
        block.style.marginBottom = '1em';
    });
});